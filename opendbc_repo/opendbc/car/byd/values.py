from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarDocs, CarParts, SupportType
from opendbc.car.lateral import AngleSteeringLimits


class BydSafetyFlags(IntFlag):
  # experimental 482 camera angle path (Atto 3 style); Song Plus DM-i does not
  # transmit 0x1E2, the default (0) is the 790 LKAS_Output torque path
  ANGLE_STEERING = 1
  # OP longitudinal control (transparent ACC_CMD/ACC_HUD/ACC_AEB replacement).
  # Firmware gates the extra TX whitelist, the 0x32E accel checks and the
  # 0x32D/E/F forward block on this flag, so toggling the param needs no
  # reflash - just an OP restart (boardd re-applies the safety param).
  LONGITUDINAL = 2


# flip to True to use the experimental angle path instead of the default torque path
USE_ANGLE_STEERING = False


class CarControllerParams:
  # --- torque path (default), matches BYD_TORQUE_STEERING_LIMITS in byd.h ---
  # Calibration anchored to the working vendor build's live traffic (route
  # 00000037): it steers this car with |torque| p50=67 / p90=128 / max=193
  # within the LKAS_Config=3 session, and its per-frame steps cap at ~16
  # (@50 Hz, its own firmware limit 17). Earlier "50-62 ceiling" readings
  # were plausibility rules on mis-marked (Config=2/State=2) frames, not the
  # real envelope. Our firmware safety limits (300/10/12) stay untouched as
  # upper bounds; deltas 8/10 fit under them.
  STEER_MAX = 200
  STEER_STEP = 2            # 50 Hz command rate (100 Hz control loop)
  STEER_DELTA_UP = 8        # per 50 Hz command; firmware safety allows 10
  STEER_DELTA_DOWN = 10     # per 50 Hz command; firmware safety allows 12
  STEER_DRIVER_ALLOWANCE = 68
  STEER_DRIVER_MULTIPLIER = 3
  STEER_DRIVER_FACTOR = 1
  STEER_ERROR_MAX = 50
  STEER_SOFTSTART_STEP = 10  # per command; full 0->200 ramp takes ~0.5 s (STEER_DELTA_UP 8 binds, not this; vendor ramps ~9/frame)
  # Drive 2 (hour_logs_2) EPS latch rules, byte-proven 6/6: an armed
  # (Config=3/Active=1) session latches TorqueFailed when its request sits at
  # ~zero for ~0.5 s (SteerWarning fires ~0.2 s in first; the 6 latches came
  # 0.48-0.72 s after the request hit zero) or when the request OPPOSES the
  # driver (R16_000: -70 vs +46, under the old 68 allowance). The vendor build
  # never does either - its request follows the driver, nonzero, through
  # drv>150 fights. These gate both failure modes at the controller.
  STEER_SILENCE_FRAMES = 8    # 50 Hz commands: ~0.16 s of armed |request| < 2 -> exit session
  # Opposition is measured from pre-clip DEMAND: the driver-limit clip zeroes
  # opposing output past |drv| ~135 (68 + STEER_MAX/3), so an output-based
  # test goes blind exactly in heavy fights (drive 2 measured 111-240).
  # 15 sits inside the hands-off noise band (<50, light grip 60-150) - the
  # sustained counter is the noise filter; raise only with road data (R16
  # shows opposition is rejected from drv ~18 up).
  STEER_DRIVER_OPPOSING = 15  # raw EPS driver torque: sustained demand opposition flips to follow
  STEER_FOLLOW_TORQUE = 20    # same-direction follow request while the driver has the wheel
  # Vendor yield curve (decrypted op_byd apply_byd_steer_torque_limits): the
  # request is scaled down smoothly as |driver torque| grows instead of the
  # binary clip. Applied on the NORMAL path only - opposing requests still go
  # through the detection-window/follow machinery below, so a scaled-to-zero
  # opposing request (armed-silence) can never be emitted and the driver-limit
  # clip zero (68 + 200/3 ~ 186.7) is unreachable by construction. Endpoint
  # 186 keeps the curve zero co-located with that clip zero.
  STEER_YIELD_DRV_BP = [50., 120., 186.]
  STEER_YIELD_FACTOR = [1.0, 0.5, 0.0]
  # EPS LKAS fault gates, all measured on mis-marked frames (before the
  # Config=3 session fix). They never fired in the vendor's clean steering
  # (which has no such gates), so these are extra conservatism on top of the
  # vendor-proven envelope - revisit if they feel intrusive.
  STEER_ANGLE_GATE_DEACT = 40.   # deg, stand down above this
  STEER_ANGLE_GATE_REARM = 30.   # deg, allow requests again below this
  # Re-arming additionally requires the wheel SETTLED: route 00000027 seg 1
  # faulted when a parking-turn unwind swept the wheel through center at
  # ~150 deg/s - the instantaneous re-arm fired at the 0-crossing and the EPS
  # latched TorqueFailed on a request into a full-lock-speed swing. Normal
  # driving in the same log peaked at 129 deg/s, so re-arm needs < 40 deg/s
  # held for 0.3 s; violent swings (> 180 deg/s) stand down at any time.
  STEER_RATE_DEACT = 180.        # deg/s, stand down above this at any time
  STEER_RATE_REARM = 40.         # deg/s, re-arm requires rate below this
  STEER_ANGLE_SETTLE_FRAMES = 30  # 0.3 s at the 100 Hz control loop
  # Route 0000002a fault 5: all five TorqueFailed latches happened seconds
  # after a >50 deg excursion (parking-turn unwind), while two long clean
  # windows were pure forward driving - the EPS refuses LKAS requests for a
  # while after the wheel has been far off center (the stock camera's own LKAS
  # state machine does the same). Lock requests out for 10 s after the last
  # >50 deg sample, on top of the settle requirement.
  STEER_LARGE_ANGLE = 50.        # deg, the proven fault line
  STEER_LARGE_ANGLE_LOCKOUT = 10.  # s after the last large-angle sample
  # driver torque for steeringPressed (raw EPS scale: hands-off noise <50,
  # light grip 60-150)
  STEER_THRESHOLD = 80  # TODO(Song Plus DM-i): calibrate

  # --- longitudinal (OP ACC_CMD), from the decrypted op_byd build and its
  # real-vehicle TX frames (route 00000037, docs_site/op_byd_logs) ---
  ACCEL_MIN = -4.0           # vendor ACCEL_MIN (opendbc base is -3.5)
  ACCEL_MAX = 2.0            # vendor ACCEL_MAX; raw 140 = +2.0 was the TX cap
  COMFORT_BAND_UPPER = 0.1   # sent only while accel >= 0 (route 37: band 0.1/0.05)
  COMFORT_BAND_LOWER = 0.05
  JERK_UPPER_LIMIT = 1.0     # raw 5; 692/697 active frames in route 37
  JERK_LOWER_LIMIT = -0.8    # raw 46; vendor clip(jerk, -4, -0.8) with no plan
                             # jerk available to us, so the calm-floor value
  MIN_START_ACCEL = 0.2      # vendor: can_accel = max(0.2, can_accel) on resume
  STOP_ACCEL = -4.0          # vendor: adas TOO_CLOSE -> ACCEL_MIN

  # --- angle path (experimental) ---
  # DiPilot faults when the steering request exceeds ~90 degrees,
  # and the angle command is sent at 0.1 deg/bit
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    90.,  # deg, DiPilot faults above this  # TODO(Song Plus DM-i): calibrate from real vehicle data
    ([0., 5., 15.], [3., 1.2, 0.35]),
    ([0., 5., 15.], [3., 2.5, 0.6]),
  )

  def __init__(self, CP):
    pass


@dataclass
class BYDCarDocs(CarDocs):
  package: str = "DiPilot"
  car_parts: CarParts = field(default_factory=CarParts)
  # community port, not validated by comma
  support_type: SupportType = SupportType.COMMUNITY


@dataclass(frozen=True)
class BYDCarSpecs(CarSpecs):
  # specs cross-checked against the working vendor build's live carParams
  # (route 00000037): mass=1926, steerRatio=19.5
  centerToFrontRatio: float = 0.44
  steerRatio: float = 19.5
  mass: float = 1926.


@dataclass
class BYDPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'byd_general_pt'})


class CAR(Platforms):
  BYD_SONG_PLUS_DMI_22 = BYDPlatformConfig(
    [BYDCarDocs("BYD Song Plus DM-i 2022")],
    BYDCarSpecs(mass=1926., wheelbase=2.765),
  )


DBC = CAR.create_dbc_map()

# ratio between the wheel speed sensor and the real speed; calibrate from logs
HUD_MULTIPLIER = 1.0
