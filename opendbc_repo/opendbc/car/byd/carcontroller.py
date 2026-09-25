from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.byd import bydcan
from opendbc.car.byd.values import CarControllerParams

VisualAlert = structs.CarControl.HUDControl.VisualAlert

RES_INTERVAL = 125   # frames between resume presses (100 Hz loop)
SNG_WAIT = 310       # frames to wait before first resume press
RES_LEN = 3          # number of resume presses


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.packer = CANPacker(dbc_names[Bus.pt])

    self.apply_angle_last = 0
    self.lka_active = False
    self.is_sng_check = False
    self.sng_next_press_frame = 0  # frame where the next resume press is allowed
    self.resume_counter = 0        # counter for tracking the progress of a resume press
    self.lead_valid = False

  def update(self, CC, CC_SP, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl

    can_sends = []

    ### STEER ###
    apply_angle = apply_std_steer_angle_limits(actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                               CS.out.steeringAngleDeg, CC.latActive, CarControllerParams.ANGLE_LIMITS)

    # BYD CAN controlled lateral runs at 50 Hz
    if self.frame % 2 == 0:
      # logic to activate and deactivate lane keep; cannot tie to the lka_on state
      # because it will occasionally deactivate itself
      if CS.lka_on:
        self.lka_active = True
      if not CS.lka_on and CS.lkas_rdy_btn:
        self.lka_active = False

      if CS.out.steeringTorqueEps > 15:
        apply_angle = CS.out.steeringAngleDeg

      lat_active = CC.latActive and self.lka_active and not CS.out.standstill
      can_sends.append(bydcan.create_can_steer_command(
        self.packer, apply_angle, lat_active, CS.out.standstill, (self.frame // 2) % 16))
      can_sends.append(bydcan.create_lkas_hud(
        self.packer, CC.enabled, CS.lss_state, CS.lss_alert, CS.tsr, CS.ahb, CS.passthrough, CS.hma, CS.pt2, CS.pt3,
        CS.pt4, CS.pt5, self.lka_active, self.frame % 16))

    ### SNG auto resume ###
    auto_resume_allowed = CC.enabled and CS.out.cruiseState.standstill

    if not auto_resume_allowed:
      self.is_sng_check = False
    else:
      self.lead_valid = hud_control.leadVisible and self.lead_valid

      if not self.is_sng_check:
        # SNG auto resume check start
        self.is_sng_check = True
        self.lead_valid = True
        self.sng_next_press_frame = self.frame + SNG_WAIT
        self.resume_counter = 0

      elif self.resume_counter >= RES_LEN or CS.out.gasPressed or CS.res_btn_pressed:
        # auto resume finished or manual press
        self.sng_next_press_frame = max(self.sng_next_press_frame, self.frame + RES_INTERVAL)
        self.resume_counter = 0

      elif self.lead_valid and self.frame > self.sng_next_press_frame:
        # send resume press signal
        can_sends.append(bydcan.send_buttons(self.packer, 1, (CS.counter_pcm_buttons + 1) % 16))
        self.resume_counter += 1

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = apply_angle

    self.frame += 1
    return new_actuators, can_sends
