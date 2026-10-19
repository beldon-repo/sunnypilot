from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.byd.carcontroller import CarController
from opendbc.car.byd.carstate import CarState
from opendbc.car.byd.radar_interface import RadarInterface
from opendbc.car.byd.values import HUD_MULTIPLIER, USE_ANGLE_STEERING, BydSafetyFlags, CarControllerParams


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  def update(self, can_packets):
    ret, ret_sp = super().update(can_packets)
    # T8 gap monitor rides the bus1 parser owned by CarState (see the note in
    # carstate init: deliberately outside can_parsers, canValid/canTimeout are
    # NOT affected). Fields are telemetry until desire_helper gates on them.
    ret.radarGapClosing, ret.radarGapNew = self.CS.update_radar_gap(can_packets, ret.vEgo)
    return ret, ret_sp

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

    # Radar interface LIVE (2026-10-19, alpha-longitudinal verification round):
    # primary synthesized lead = continuous 0x109 b7 * BydRadarDistScale (pinned
    # 0.25 m/count) with the presence==7 gate and median-filtered vRel;
    # 0x351-354 slot path stays as the auto-lighting future source (private-CAN
    # tap), 0x32D HUD bars as the last-resort fallback. In stock-ACC mode
    # radarState feeds nothing longitudinal (vendors' MPC keeps owning the
    # session), so this flag is behavior-neutral there; with
    # AlphaLongitudinalEnabled the synthesized lead feeds the longitudinal MPC
    # and this road test validates dRel/vRel quality end-to-end. The gap
    # monitor (BydRadarGapGate) is unaffected (counts space, own parser).
    ret.radarUnavailable = False

    # longitudinal control is done by the stock ACC; openpilot only does
    # lateral control plus spoofed resume button for auto-resume from standstill.
    # pcmCruise ties openpilot's engagement to the ACC main posture (like the
    # Geely port): cruiseState.enabled is the arm latch (debounced main-on +
    # one genuine session this drive, see carstate) - OP enables on its rising
    # edge and only drops it when the driver turns ACC off. Brake and session
    # standby deliberately do not disengage; the longitudinal output is gated
    # on the live radar session in the controller instead.
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
    # Declare availability: without this flag selfdrived deletes
    # AlphaLongitudinalEnabled at every startup and exp_button locks the onroad
    # toggle ("stock ACC is used" message). Running the param on firmware
    # predating the longitudinal port (fw_base fee17db6) is instead caught by
    # the safetyTxBlocked watchdog (bydLongFirmwareMissing, M1).
    ret.alphaLongitudinalAvailable = True
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
