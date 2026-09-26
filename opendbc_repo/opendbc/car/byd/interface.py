from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.byd.carcontroller import CarController
from opendbc.car.byd.carstate import CarState
from opendbc.car.byd.values import HUD_MULTIPLIER, USE_ANGLE_STEERING, BydSafetyFlags


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
      ret.steerActuatorDelay = 0.1
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)
    ret.steerLimitTimer = 0.4

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

    return ret
