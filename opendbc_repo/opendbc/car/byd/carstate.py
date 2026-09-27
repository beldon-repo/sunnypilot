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
    self.is_cruise_latch = False
    # stock camera's ACC_MPC_STATE (bus 2); echoed back on bus 0 by the
    # controller so the spoofed LKAS request keeps the camera's SETME_* fields
    self.cam_lkas = {}

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]
    cp_adas = can_parsers[Bus.adas]

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # EPS feedback (also read back by the ADAS domain)
    self.lkas_prepared = bool(cp.vl["ACC_EPS_STATE"]["LKAS_Prepared"])
    self.torque_failed = bool(cp.vl["ACC_EPS_STATE"]["TorqueFailed"])
    self.eps_state_msg = cp.vl["ACC_EPS_STATE"]

    # stock camera's ACC_MPC_STATE (bus 2); the controller echoes this frame
    # onto bus 0 with the LKAS torque overridden. Echoing the camera's SETME_*
    # / MPC_State fields is required - building the frame from scratch faults
    # the DiPilot ADAS domain ('check multifunction video controller').
    self.cam_lkas = cp_adas.vl["ACC_MPC_STATE"]

    # speed
    # Song Plus DM-i: the 0x122 wheel-speed message reads all zeros and the
    # 0x418 BSD_RADAR VEHICLE_SPEED field is constant (~8.5), so neither is a
    # usable speed. The real vehicle speed comes from 0x1f0 ESP_SPEED
    # (20 Hz, bus 0) byte 4 in km/h, verified against camera odometry on real
    # drives (regression slope ~3.6, see route logs 2-7).
    vehicle_speed_kph = cp.vl["ESP_SPEED"]["VehicleSpeed"]
    self.parse_wheel_speeds(ret,
      vehicle_speed_kph,
      vehicle_speed_kph,
      vehicle_speed_kph,
      vehicle_speed_kph,
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
    # the car already enforces the seatbelt warning itself; OP does not need to
    # block engagement on it. The DriverSeatBeltFasten bit encoding is also
    # unverified on Song Plus DM-i (would otherwise report unlatched while
    # buckled). Matches the community BYD_Files port.
    ret.seatbeltUnlatched = False

    # pedals
    ret.gasPressed = cp.vl["PEDAL"]["AcceleratorPedal"] > 0.01
    ret.brake = cp.vl["PEDAL"]["BrakePedal"]
    ret.brakePressed = bool(cp.vl["DRIVE_STATE"]["BrakePressed"]) or ret.brake > 0.01

    # steering
    ret.steeringAngleDeg = cp.vl["EPS"]["SteeringAngle"]
    # EPS-reported wheel rate in deg/s (4 deg/s/bit, 0-1020); used by the LKAS
    # envelope gate to tell a settled wheel from a full-lock-speed swing
    ret.steeringRateDeg = cp.vl["EPS"]["SteeringAngleRate"]
    ret.steeringTorque = cp.vl["ACC_EPS_STATE"]["SteerDriverTorque"]
    ret.steeringTorqueEps = cp.vl["ACC_EPS_STATE"]["MainTorque"]
    # 5-frame debounce: single-frame torque noise spikes must not flap
    # steeringPressed (they would keep selfdrived in overriding)
    ret.steeringPressed = self.update_steering_pressed(
      bool(abs(ret.steeringTorque) > CarControllerParams.STEER_THRESHOLD), 5)

    # stock ACC status; LKA is coupled to the stock ACC on this platform
    # Song Plus DM-i encodes AccState differently from Han: 1 = ACC_ACTIVE,
    # 7 = main on / standby (observed with SetSpeed=30, AccOn1=1)
    acc_state = int(cp_adas.vl["ACC_HUD_ADAS"]["AccState"])
    acc_on1 = bool(cp_adas.vl["ACC_HUD_ADAS"]["AccOn1"])
    ret.cruiseState.available = acc_on1 or acc_state in (1, 2, 3, 5)
    set_speed = cp_adas.vl["ACC_HUD_ADAS"]["SetSpeed"]
    # follow the stock ACC set speed directly (community port does the same);
    # no artificial 30 km/h floor
    ret.cruiseState.speedCluster = set_speed * CV.KPH_TO_MS if ret.cruiseState.available else 0.
    ret.cruiseState.speed = ret.cruiseState.speedCluster

    acc_control_active = bool(cp_adas.vl["ACC_CMD"]["AccControlActive"])
    standstill_state = bool(cp_adas.vl["ACC_CMD"]["StandstillState"])
    ret.cruiseState.standstill = standstill_state
    ret.cruiseState.nonAdaptive = False

    self.res_btn_pressed = cp.vl["PCM_BUTTONS"]["BTN_AccUpDown_Cmd"] != 0
    self.counter_pcm_buttons = cp.vl["PCM_BUTTONS"]["Counter"]

    # cruiseState.enabled must strictly track real stock-ACC engagement: the
    # pcmEnable rising edge on `enabled` is what engages openpilot, so a
    # phantom-true at ignition eats the edge and ACC-on-then-SET never
    # produces a second one. AccState=1 is ambiguous on Song Plus DM-i - the
    # camera reports it at ignition (leftover state, SetSpeed=30, still in
    # Park) and again after a brake disengage, while it is not commanding the
    # car. AccControlActive (0x32e) is the reliable engagement signal observed
    # on real drives: 0 at ignition-leftover/standby, 1 whenever the MPC
    # actually commands ACC, including the SNG standstill hold
    # (StandstillState=1). AccState 2/3/5 are engaged-only states.
    stock_acc_on = acc_control_active or acc_state in (2, 3, 5)
    if not ret.cruiseState.available or ret.brakePressed or not stock_acc_on:
      self.is_cruise_latch = False
    else:
      self.is_cruise_latch = True

    ret.cruiseState.enabled = self.is_cruise_latch
    # stock LKA is coupled to the stock ACC on this platform; used by the
    # experimental angle path for engagement gating
    self.lka_on = ret.cruiseState.enabled

    # stalks and blind spots
    ret.leftBlinker = bool(cp.vl["STALKS"]["LeftIndicator"])
    ret.rightBlinker = bool(cp.vl["STALKS"]["RightIndicator"])
    ret.espDisabled = False
    ret.stockAeb = bool(cp_adas.vl["ACC_HUD_ADAS"]["AEB"])
    ret.stockFcw = bool(cp_adas.vl["ACC_HUD_ADAS"]["FCW"])

    ret.leftBlindspot = bool(cp.vl["BSD_RADAR"]["LEFT_APPROACH"])
    ret.rightBlindspot = bool(cp.vl["BSD_RADAR"]["RIGHT_APPROACH"])

    # The EPS latches TorqueFailed and gives up all steering input until it is
    # power-cycled (real vehicle, route 0000001c/20/22; the independent
    # yysnet/opendbc port documents the same behavior). Surface it so selfdrived
    # alerts the driver and blocks engagement instead of silently not steering;
    # the controller additionally stands down on the same bit.
    ret.steerFaultPermanent = bool(cp.vl["ACC_EPS_STATE"]["TorqueFailed"])

    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    # Song Plus DM-i bus layout (verified from real vehicle CAN logs):
    #   bus0 = powertrain/chassis + ADAS feedback (EPS, BSD_RADAR, ACC_EPS_STATE, ...)
    #   bus2 = ACC/ADAS command domain (ACC_HUD_ADAS, ACC_CMD)
    dbc = DBC[CP.carFingerprint][Bus.pt]
    pt_messages = [
      ("EPS", 100),              # 0x11f
      ("WHEEL_SPEED", 50),       # 0x122
      ("ESP_SPEED", 20),         # 0x1f0, real vehicle speed in km/h
      ("BCM", 10),               # 0x12d
      ("STALKS", 10),            # 0x133
      ("DRIVE_STATE", 20),       # 0x242
      ("ACC_EPS_STATE", 50),     # 0x318
      ("PEDAL", 20),             # 0x342
      ("PCM_BUTTONS", 10),       # 0x3b0
      ("BSD_RADAR", 10),         # 0x418
    ]
    adas_messages = [
      ("ACC_HUD_ADAS", 20),      # 0x32d
      ("ACC_CMD", 20),           # 0x32e
      ("ACC_MPC_STATE", 50),     # 0x316, stock camera LKAS frame (echoed on bus 0)
    ]
    return {
      Bus.pt: CANParser(dbc, pt_messages, 0),
      Bus.adas: CANParser(dbc, adas_messages, 2),
    }
