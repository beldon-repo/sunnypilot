from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL
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
    self.angle_gate = False
    self.angle_settle_counter = 0
    self.last_large_angle_frame = -10 ** 9  # far past: no boot-time lockout

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
    # EPS LKAS operating envelope (values.py fault map): requesting outside it
    # latches TorqueFailed until ignition-off, killing lateral for the whole
    # drive. Proven triggers: large |steering angle| (3 reproductions) and any
    # request during a full-lock-speed swing (route 00000027 seg 1: a parking
    # unwind swept the wheel through center at ~150 deg/s, the instantaneous
    # re-arm fired at the 0-crossing, and -48 units into that swing latched
    # the EPS). So: stand down above the angle/rate limits, and re-arm only
    # after the wheel has SETTLED - small angle and low rate, held.
    ang = abs(CS.out.steeringAngleDeg)
    rate = abs(CS.out.steeringRateDeg)
    if ang > CarControllerParams.STEER_LARGE_ANGLE:
      # past the EPS fault line: close the gate and stamp the excursion time
      self.angle_gate = True
      self.angle_settle_counter = 0
      self.last_large_angle_frame = self.frame
    elif ang > CarControllerParams.STEER_ANGLE_GATE_DEACT or rate > CarControllerParams.STEER_RATE_DEACT:
      self.angle_gate = True
      self.angle_settle_counter = 0
    elif self.angle_gate and ang < CarControllerParams.STEER_ANGLE_GATE_REARM and rate < CarControllerParams.STEER_RATE_REARM \
          and (self.frame - self.last_large_angle_frame) * DT_CTRL > CarControllerParams.STEER_LARGE_ANGLE_LOCKOUT:
      self.angle_settle_counter += 1
      if self.angle_settle_counter >= CarControllerParams.STEER_ANGLE_SETTLE_FRAMES:
        self.angle_gate = False
    elif not (ang < CarControllerParams.STEER_ANGLE_GATE_REARM and rate < CarControllerParams.STEER_RATE_REARM):
      self.angle_settle_counter = 0

    lat_active = CC.latActive and not self.lkas_brake_inhibit and not CS.out.standstill \
      and not self.angle_gate

    # A latched EPS TorqueFailed (real drive, route 0000001c seg 0: fault fired
    # ~0.7 s into the torque ramp while the driver resisted, then stayed
    # latched) invalidates everything - stand down to plain idle echo, no
    # LKAS_Active and no ReqPrepare, until the EPS clears it. Never treat a
    # stale LKAS_Prepared as armed across the fault.
    if CS.torque_failed:
      self.lkas_active = False

    # engage only when the EPS reports the stock LKA prepared; deactivate otherwise
    if lat_active and not self.lkas_active and not CS.torque_failed:
      if CS.lkas_prepared:
        self.lkas_active = True
        self.steer_softstart_limit = 0
    elif not lat_active:
      self.lkas_active = False

    lkas_req_prepare = 0
    if self.lkas_active:
      # compute at the 50 Hz command rate so the per-command delta limits match
      # the firmware safety model exactly (the control loop runs at 100 Hz)
      if self.frame % 2 == 0:
        # actuators.torque is normalized to [-1, 1]
        new_torque = int(round(CC.actuators.torque * CarControllerParams.STEER_MAX))
        new_torque = min(max(new_torque, -self.steer_softstart_limit, -CarControllerParams.STEER_MAX),
                         self.steer_softstart_limit)
        if self.steer_softstart_limit < CarControllerParams.STEER_MAX:
          self.steer_softstart_limit += CarControllerParams.STEER_SOFTSTART_STEP

        self.apply_torque_last = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                                  CS.out.steeringTorque, CarControllerParams)
    else:
      self.apply_torque_last = 0
      if lat_active and not CS.lkas_prepared and not CS.torque_failed:
        # ask the EPS to arm LKA; it responds with LKAS_Prepared in ACC_EPS_STATE
        lkas_req_prepare = 1

    # 50 Hz; echo the stock camera's ACC_MPC_STATE so the SETME_* / MPC_State
    # fields match what the DiPilot ADAS domain expects (from-scratch frames
    # fault the camera). Skip until we have seen the camera's 0x316.
    if self.frame % 2 == 0 and CS.cam_lkas:
      # LKAS_Config=3 (ALARM_AND_LKA) unconditionally - it is the EPS's
      # actuation-permission session mode. The stock camera never sends it
      # (its idle is 1/ALARM or 2), which is why echoed-Config frames arm,
      # echo back, stay fault-free - and never actuate. Matches the working
      # vendor build byte-for-byte (route 00000037).
      lkas_config = 3
      return bydcan.create_lkas_request(
        self.packer, CS.cam_lkas, self.apply_torque_last, self.lkas_active,
        lkas_req_prepare, lkas_config, (self.frame // 2) % 16)
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

    # Transmit the spoofed 0x316 at 50 Hz UNCONDITIONALLY (engaged or not).
    # The camera's own 0x316 is blocked from relaying bus2->bus0 by the
    # firmware fwd hook, so OP is the EPS's only 0x316 source - if we stop
    # transmitting while disengaged, the EPS LKAS subsystem starves and
    # faults the ADAS domain ('check multifunction video controller', EPS
    # SteerWarning latched in real-vehicle logs). Idle frames echo the
    # camera's fields with LKAS_Active=0 / torque 0. The old claim that
    # continuous TX faults the domain only applied to the pre-echo
    # from-scratch frames (missing SETME_* fields). Matches the
    # community-verified BYD_Files controller, which transmits every cycle
    # regardless of engagement.
    steer_send = None
    new_actuators = CC.actuators.as_builder()
    if USE_ANGLE_STEERING:
      if CC.enabled or CC.latActive:
        steer_send = self._update_angle_lateral(CC, CS)
        if steer_send is not None:
          new_actuators.steeringAngleDeg = self.apply_angle_last
    else:
      steer_send = self._update_torque_lateral(CC, CS)
      new_actuators.torque = self.apply_torque_last / CarControllerParams.STEER_MAX
      new_actuators.torqueOutputCan = float(self.apply_torque_last)

    if steer_send is not None:
      can_sends.append(steer_send)

    self._update_sng_auto_resume(CC, CS, can_sends)

    self.frame += 1
    return new_actuators, can_sends
