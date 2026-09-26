from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.lateral import apply_driver_steer_torque_limits, apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.byd import bydcan
from opendbc.car.byd.values import USE_ANGLE_STEERING, CarControllerParams

RES_INTERVAL = 125   # frames between resume presses (100 Hz loop)
SNG_WAIT = 310       # frames to wait before first resume press
RES_LEN = 3          # number of resume presses

BRAKE_INHIBIT_PRESSED_THRESHOLD = 3   # frames of brake press before inhibiting lateral
BRAKE_INHIBIT_RELEASE_THRESHOLD = 5   # frames of brake release before re-enabling


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.packer = CANPacker(dbc_names[Bus.pt])

    # lateral state
    self.apply_torque_last = 0
    self.apply_angle_last = 0
    self.lkas_active = False
    self.steer_softstart_limit = 0
    self.lkas_brake_inhibit = False
    self.brake_pressed_counter = 0
    self.brake_release_counter = 0

    # SNG auto-resume state
    self.is_sng_check = False
    self.sng_next_press_frame = 0
    self.resume_counter = 0
    self.lead_valid = False

  def _handle_brake_inhibit(self, CS):
    """Inhibit lateral control while the driver brakes (avoids LKS faults), with debounce."""
    if CS.out.brakePressed:
      self.brake_release_counter = 0
      if not self.lkas_brake_inhibit:
        self.brake_pressed_counter += 1
        if self.brake_pressed_counter >= BRAKE_INHIBIT_PRESSED_THRESHOLD:
          self.lkas_brake_inhibit = True
    else:
      self.brake_pressed_counter = 0
      if self.lkas_brake_inhibit:
        self.brake_release_counter += 1
        if self.brake_release_counter >= BRAKE_INHIBIT_RELEASE_THRESHOLD:
          self.lkas_brake_inhibit = False

  def _update_torque_lateral(self, CC, CS):
    """Default torque path: steer via the LKAS_Output request in ACC_MPC_STATE (790)."""
    lat_active = CC.latActive and not self.lkas_brake_inhibit and not CS.out.standstill

    # engage only when the EPS reports the stock LKA prepared; deactivate otherwise
    if lat_active and not self.lkas_active:
      if CS.lkas_prepared:
        self.lkas_active = True
        self.steer_softstart_limit = 0
    elif not lat_active:
      self.lkas_active = False

    apply_torque = 0
    if self.lkas_active:
      # actuators.torque is normalized to [-1, 1]
      new_torque = int(round(CC.actuators.torque * CarControllerParams.STEER_MAX))
      new_torque = min(max(new_torque, -self.steer_softstart_limit, -CarControllerParams.STEER_MAX),
                       self.steer_softstart_limit)
      if self.steer_softstart_limit < CarControllerParams.STEER_MAX:
        self.steer_softstart_limit += CarControllerParams.STEER_SOFTSTART_STEP

      apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                      CS.out.steeringTorque, CarControllerParams)

    self.apply_torque_last = apply_torque

    # 50 Hz
    if self.frame % 2 == 0:
      # 2=LKA when active, 1=ALARM otherwise
      lkas_config = 2 if self.lkas_active else 1
      return bydcan.create_lkas_request(
        self.packer, self.apply_torque_last, self.lkas_active, lkas_config, (self.frame // 2) % 16)
    return None

  def _update_angle_lateral(self, CC, CS):
    """Experimental angle path: steer via the STEERING_MODULE_ADAS (482) angle request."""
    apply_angle = apply_std_steer_angle_limits(CC.actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                               CS.out.steeringAngleDeg, CC.latActive, CarControllerParams.ANGLE_LIMITS)

    # engage logic tied to the stock LKA button state
    if CS.lka_on:
      self.lkas_active = True
    if not CS.lka_on and not CS.out.cruiseState.enabled:
      self.lkas_active = False

    if CS.out.steeringTorqueEps > 15:
      apply_angle = CS.out.steeringAngleDeg

    lat_active = CC.latActive and self.lkas_active and not CS.out.standstill

    # 50 Hz
    if self.frame % 2 == 0:
      return bydcan.create_can_steer_command(
        self.packer, apply_angle, lat_active, CS.out.standstill, (self.frame // 2) % 16)
    return None

  def _update_sng_auto_resume(self, CC, CS, can_sends):
    """Spoof the ACC resume button while stationary and a lead is detected."""
    auto_resume_allowed = CC.enabled and CS.out.cruiseState.standstill

    if not auto_resume_allowed:
      self.is_sng_check = False
    else:
      self.lead_valid = CC.hudControl.leadVisible and self.lead_valid

      if not self.is_sng_check:
        self.is_sng_check = True
        self.lead_valid = True
        self.sng_next_press_frame = self.frame + SNG_WAIT
        self.resume_counter = 0

      elif self.resume_counter >= RES_LEN or CS.out.gasPressed or CS.res_btn_pressed:
        self.sng_next_press_frame = max(self.sng_next_press_frame, self.frame + RES_INTERVAL)
        self.resume_counter = 0

      elif self.lead_valid and self.frame > self.sng_next_press_frame:
        can_sends.append(bydcan.send_buttons(self.packer, (CS.counter_pcm_buttons + 1) % 16))
        self.resume_counter += 1

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    self._handle_brake_inhibit(CS)

    # only inject the spoofed LKAS request while openpilot is engaged.
    # NOTE(Song Plus DM-i): the stock DiPilot camera stays connected on this
    # platform (unlike Han where OP replaces it); continuously transmitting
    # 0x316 while disengaged faults the ADAS domain (ACC_HUD_ADAS AccState=7)
    # and makes the stock ACC buttons unresponsive.
    steer_send = None
    if CC.enabled or CC.latActive:
      if USE_ANGLE_STEERING:
        steer_send = self._update_angle_lateral(CC, CS)
        new_actuators = CC.actuators.as_builder()
        if steer_send is not None:
          new_actuators.steeringAngleDeg = self.apply_angle_last
      else:
        steer_send = self._update_torque_lateral(CC, CS)
        new_actuators = CC.actuators.as_builder()
        new_actuators.torque = self.apply_torque_last / CarControllerParams.STEER_MAX
        new_actuators.torqueOutputCan = float(self.apply_torque_last)

    if steer_send is not None:
      can_sends.append(steer_send)

    self._update_sng_auto_resume(CC, CS, can_sends)

    self.frame += 1
    return new_actuators, can_sends
