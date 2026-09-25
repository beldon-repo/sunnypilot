from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.byd.values import DBC, CarControllerParams


class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["DRIVE_STATE"]["Gear"]

    self.lkas_prepared = False
    self.torque_failed = False
    self.res_btn_pressed = False
    self.counter_pcm_buttons = 0
    self.eps_state_msg = {}

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # EPS feedback (also read back by the ADAS domain)
    self.lkas_prepared = bool(cp.vl["ACC_EPS_STATE"]["LKAS_Prepared"])
    self.torque_failed = bool(cp.vl["ACC_EPS_STATE"]["TorqueFailed"])
    self.eps_state_msg = cp.vl["ACC_EPS_STATE"]

    # speed
    # TODO(Song Plus DM-i): verify wheel speed layout from real vehicle CAN logs
    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_FL"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_FR"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_BL"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_BL"],  # TODO: BR sensor quirk on Atto 3, verify on Song
    )
    ret.vEgoCluster = ret.vEgo
    ret.standstill = ret.vEgoRaw < 0.05

    ret.brakeHoldActive = False

    # gear
    can_gear = int(cp.vl["DRIVE_STATE"]["Gear"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    ret.doorOpen = any([cp.vl["BCM"]["RearLeftDoor"],
                        cp.vl["BCM"]["FrontLeftDoor"],
                        cp.vl["BCM"]["RearRightDoor"],
                        cp.vl["BCM"]["FrontRightDoor"]])
    ret.seatbeltUnlatched = cp.vl["BCM"]["DriverSeatBeltFasten"] == 0

    # pedals
    ret.gasPressed = cp.vl["PEDAL"]["AcceleratorPedal"] > 0.01
    ret.brake = cp.vl["PEDAL"]["BrakePedal"]
    ret.brakePressed = bool(cp.vl["DRIVE_STATE"]["BrakePressed"]) or ret.brake > 0.01

    # steering
    ret.steeringAngleDeg = cp.vl["EPS"]["SteeringAngle"]
    ret.steeringTorque = cp.vl["ACC_EPS_STATE"]["SteerDriverTorque"]
    ret.steeringTorqueEps = cp.vl["ACC_EPS_STATE"]["MainTorque"]
    ret.steeringPressed = bool(abs(ret.steeringTorque) > CarControllerParams.STEER_THRESHOLD)

    # stock ACC status; LKA is coupled to the stock ACC on this platform
    acc_state = int(cp.vl["ACC_HUD_ADAS"]["AccState"])
    ret.cruiseState.available = acc_state in (2, 3, 5)  # ACC_ON, ACC_ACTIVE, FORCE_ACCEL
    set_speed = cp.vl["ACC_HUD_ADAS"]["SetSpeed"]
    if ret.cruiseState.available:
      ret.cruiseState.speedCluster = max(set_speed, 30) * CV.KPH_TO_MS
    else:
      ret.cruiseState.speedCluster = 0.
    ret.cruiseState.speed = ret.cruiseState.speedCluster

    acc_control_active = bool(cp.vl["ACC_CMD"]["AccControlActive"])
    acc_req_not_standstill = bool(cp.vl["ACC_CMD"]["AccReqNotStandstill"])
    standstill_state = bool(cp.vl["ACC_CMD"]["StandstillState"])
    ret.cruiseState.standstill = standstill_state
    ret.cruiseState.nonAdaptive = False

    # BYD cancels ACC at standstill; keep track of the engaged state so openpilot
    # stays active through stops and can auto-resume with the spoofed resume button
    self.res_btn_pressed = cp.vl["PCM_BUTTONS"]["BTN_AccUpDown_Cmd"] != 0
    self.counter_pcm_buttons = cp.vl["PCM_BUTTONS"]["Counter"]
    if self.res_btn_pressed and (ret.brakePressed == standstill_state):
      self.is_cruise_latch = True

    if acc_req_not_standstill:
      self.is_cruise_latch = True
    else:
      if not standstill_state:
        self.is_cruise_latch = False

    stock_acc_on = acc_state in (2, 3, 5)
    if not ret.cruiseState.available or ret.brakePressed or not (stock_acc_on or acc_control_active):
      self.is_cruise_latch = False

    ret.cruiseState.enabled = self.is_cruise_latch
    # stock LKA is coupled to the stock ACC on this platform; used by the
    # experimental angle path for engagement gating
    self.lka_on = ret.cruiseState.enabled

    # stalks and blind spots
    ret.leftBlinker = bool(cp.vl["STALKS"]["LeftIndicator"])
    ret.rightBlinker = bool(cp.vl["STALKS"]["RightIndicator"])
    ret.espDisabled = False
    ret.stockAeb = bool(cp.vl["ACC_HUD_ADAS"]["AEB"])
    ret.stockFcw = bool(cp.vl["ACC_HUD_ADAS"]["FCW"])

    ret.leftBlindspot = bool(cp.vl["BSD_RADAR"]["LEFT_APPROACH"])
    ret.rightBlindspot = bool(cp.vl["BSD_RADAR"]["RIGHT_APPROACH"])

    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    return {
      # BYD shares one CAN bus between the powertrain and the ADAS domain
      # TODO(Song Plus DM-i): verify bus layout from real vehicle CAN logs
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], 0),
    }
