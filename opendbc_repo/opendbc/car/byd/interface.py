from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.byd.carcontroller import CarController
from opendbc.car.byd.carstate import CarState
from opendbc.car.byd.values import HUD_MULTIPLIER, USE_ANGLE_STEERING, BydSafetyFlags, CarControllerParams


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "byd"
    if USE_ANGLE_STEERING:
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.byd, BydSafetyFlags.ANGLE_STEERING)]
      ret.steerControlType = structs.CarParams.SteerControlType.angle
      ret.steerActuatorDelay = 0.01  # DiPilot steering request is applied almost instantly
    else:
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.byd)]
      ret.steerControlType = structs.CarParams.SteerControlType.torque
      # vendor live carParams (route 00000037): steerActuatorDelay=0.3
      ret.steerActuatorDelay = 0.3
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    ret.steerLimitTimer = 0.5

    ret.radarUnavailable = True

    # longitudinal control is done by the stock ACC; openpilot only does
    # lateral control plus spoofed resume button for auto-resume from standstill.
    # pcmCruise ties openpilot's engagement to the stock ACC state (like the
    # Geely port): OP enables on the rising edge of cruiseState.enabled.
    ret.openpilotLongitudinalControl = False
    ret.pcmCruise = True
    ret.autoResumeSng = True

    ret.wheelSpeedFactor = HUD_MULTIPLIER

    ret.minEnableSpeed = -1
    ret.stoppingDecelRate = 0.05  # reach stopping target smoothly

    # OP longitudinal (AlphaLongitudinalEnabled param): transparent replacement
    # of the stock radar's ACC_CMD/ACC_HUD/ACC_AEB on bus 0. The stock radar
    # keeps owning the session - its frames on bus 2 stay alive and keep feeding
    # cruiseState/pcm_cruise_check - so the engage chain above is unchanged and
    # no reflash is needed to toggle: the firmware keys its extra TX whitelist,
    # accel checks and 0x32D/E/F forward block off the LONGITUDINAL safety flag.
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= int(BydSafetyFlags.LONGITUDINAL)
      # vendor interface params (decrypted op_byd interface.py); delay to be
      # verified on the real car (alignment doc risk note)
      ret.longitudinalActuatorDelay = 0.5
      ret.vEgoStarting = 0.3
      ret.vEgoStopping = 0.2
      ret.startAccel = 0.4
      ret.stoppingDecelRate = 0.03
      # the standstill -> go transition must go through LongCtrlState.starting
      # so the controller can pulse ACC_CMD ResumeFromStandstill
      ret.startingState = True

    return ret

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    # vendor ACCEL_MIN/MAX (base class returns -3.5/2.0; BYD commands down to
    # -4.0 m/s2, see route 37 TX)
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX
