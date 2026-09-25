from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.byd.values import DBC


class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["DRIVE_STATE"]["GEAR"]

    self.prev_angle = 0
    self.is_cruise_latch = False
    # stock ADAS state that the car controller passes through on LKAS_HUD_ADAS
    self.lka_on = 0
    self.lss_state = 0
    self.lss_alert = 0
    self.tsr = 0
    self.ahb = 0
    self.passthrough = 0
    self.hma = 0
    self.pt2 = 0
    self.pt3 = 0
    self.pt4 = 0
    self.pt5 = 0
    self.lkas_rdy_btn = False
    self.res_btn_pressed = False
    self.counter_pcm_buttons = 0

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # pass through stock ADAS settings so the car controller can spoof LKAS_HUD_ADAS unchanged
    self.lka_on = cp.vl["LKAS_HUD_ADAS"]["STEER_ACTIVE_ACTIVE_LOW"]
    self.lss_state = cp.vl["LKAS_HUD_ADAS"]["LSS_STATE"]
    self.lss_alert = cp.vl["LKAS_HUD_ADAS"]["SETTINGS"]
    self.tsr = cp.vl["LKAS_HUD_ADAS"]["TSR"]
    self.ahb = cp.vl["LKAS_HUD_ADAS"]["SET_ME_XFF"]
    self.passthrough = cp.vl["LKAS_HUD_ADAS"]["SET_ME_X5F"]
    self.hma = cp.vl["LKAS_HUD_ADAS"]["HMA"]
    self.pt2 = cp.vl["LKAS_HUD_ADAS"]["PT2"]
    self.pt3 = cp.vl["LKAS_HUD_ADAS"]["PT3"]
    self.pt4 = cp.vl["LKAS_HUD_ADAS"]["PT4"]
    self.pt5 = cp.vl["LKAS_HUD_ADAS"]["PT5"]
    self.lkas_rdy_btn = bool(cp.vl["PCM_BUTTONS"]["LKAS_ON_BTN"])
    self.counter_pcm_buttons = cp.vl["PCM_BUTTONS"]["COUNTER"]
    self.res_btn_pressed = cp.vl["PCM_BUTTONS"]["SET_BTN"] != 0 or cp.vl["PCM_BUTTONS"]["RES_BTN"] != 0

    # TODO(Song Plus DM-i): verify wheel speed sensors; on Atto 3 the BR sensor makes the value wrong
    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_FL"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_FR"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_BL"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_BL"],
    )
    ret.vEgoCluster = ret.vEgo
    ret.standstill = ret.vEgoRaw < 0.05

    ret.brakeHoldActive = False

    # gear
    can_gear = int(cp.vl["DRIVE_STATE"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    ret.doorOpen = any([cp.vl["METER_CLUSTER"]["BACK_LEFT_DOOR"],
                        cp.vl["METER_CLUSTER"]["FRONT_LEFT_DOOR"],
                        cp.vl["METER_CLUSTER"]["BACK_RIGHT_DOOR"],
                        cp.vl["METER_CLUSTER"]["FRONT_RIGHT_DOOR"]])
    ret.seatbeltUnlatched = cp.vl["METER_CLUSTER"]["SEATBELT_DRIVER"] == 0

    # pedals
    ret.gasPressed = cp.vl["PEDAL"]["GAS_PEDAL"] > 0.01
    ret.brake = cp.vl["PEDAL"]["BRAKE_PEDAL"]
    ret.brakePressed = bool(cp.vl["DRIVE_STATE"]["BRAKE_PRESSED"]) or ret.brake > 0.01

    # steering
    ret.steeringAngleDeg = cp.vl["STEER_MODULE_2"]["STEER_ANGLE_2"]
    steer_dir = 1 if (ret.steeringAngleDeg - self.prev_angle >= 0) else -1
    self.prev_angle = ret.steeringAngleDeg
    ret.steeringTorque = cp.vl["STEERING_TORQUE"]["MAIN_TORQUE"]
    ret.steeringTorqueEps = cp.vl["STEER_MODULE_2"]["DRIVER_EPS_TORQUE"] * steer_dir
    ret.steeringPressed = bool(abs(ret.steeringTorqueEps) > 6)

    # stock ACC status
    ret.cruiseState.available = any([cp.vl["ACC_HUD_ADAS"]["ACC_ON1"], cp.vl["ACC_HUD_ADAS"]["ACC_ON2"]])
    set_speed = cp.vl["ACC_HUD_ADAS"]["SET_SPEED"]
    if ret.cruiseState.available:
      ret.cruiseState.speedCluster = max(set_speed, 30) * CV.KPH_TO_MS
    else:
      ret.cruiseState.speedCluster = 0.
    ret.cruiseState.speed = ret.cruiseState.speedCluster
    ret.cruiseState.standstill = bool(cp.vl["ACC_CMD"]["STANDSTILL_STATE"])
    ret.cruiseState.nonAdaptive = False

    # BYD cancels ACC at standstill; keep track of the engaged state so openpilot
    # stays active through stops and can auto-resume with the spoofed RES button
    if self.res_btn_pressed and (ret.brakePressed == ret.cruiseState.standstill):
      self.is_cruise_latch = True

    if bool(cp.vl["ACC_CMD"]["ACC_REQ_NOT_STANDSTILL"]):
      self.is_cruise_latch = True
    else:
      if not ret.cruiseState.standstill:
        self.is_cruise_latch = False

    stock_acc_on = bool(cp.vl["ACC_CMD"]["ACC_CONTROLLABLE_AND_ON"])
    if not ret.cruiseState.available or ret.brakePressed or not stock_acc_on:
      self.is_cruise_latch = False

    ret.cruiseState.enabled = self.is_cruise_latch

    # stalks and blind spots
    ret.leftBlinker = bool(cp.vl["STALKS"]["LEFT_BLINKER"])
    ret.rightBlinker = bool(cp.vl["STALKS"]["RIGHT_BLINKER"])
    ret.espDisabled = False
    ret.stockAeb = False
    ret.stockFcw = False

    ret.leftBlindspot = bool(cp.vl["BSM"]["LEFT_APPROACH"])
    ret.rightBlindspot = bool(cp.vl["BSM"]["RIGHT_APPROACH"])

    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    return {
      # BYD shares one CAN bus between the powertrain and the ADAS camera
      # TODO(Song Plus DM-i): verify bus layout from real vehicle CAN logs
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
    }
