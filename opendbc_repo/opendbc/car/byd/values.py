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
  STEER_MAX = 300
  STEER_STEP = 2            # 50 Hz command rate (100 Hz control loop)
  STEER_DELTA_UP = 8        # per 50 Hz command; firmware safety allows 10
  STEER_DELTA_DOWN = 10     # per 50 Hz command; firmware safety allows 12
  STEER_DRIVER_ALLOWANCE = 120
  STEER_DRIVER_MULTIPLIER = 3
  STEER_DRIVER_FACTOR = 1
  STEER_ERROR_MAX = 50
  # STEER_MAX/ALLOWANCE are the vendor's own firmware numbers (STEER_MAX=300,
  # ALLOWANCE=120, decrypted op_byd): the vendor reaches -153 against a
  # +166 driver yank and 193 absolute in clean steering, which the standard
  # driver-limit formula only permits with (300, 120) - under (200, 68) any
  # opposing request clips to ZERO once |drv| > 134.7, which is exactly the
  # armed-silence the EPS latches on. Our firmware byd.h caps at 300/10/12
  # already; its driver_torque_allowance must match this 120.
  # Armed-silence backstop: our instrumented sessions latched TorqueFailed
  # after 0.48-0.72 s of armed |request| ~ 0 (drive 2, 6/6). The vendor never
  # sits there - not because it gates anything, but because its demand loop
  # is always steering. Exit to the retry burst just before the measured
  # latch band; the ReqPrepare burst itself is a stream the EPS sees between
  # every vendor session and never objects to.
  STEER_ZERO_EXIT_FRAMES = 21  # 50 Hz commands: ~0.42 s armed at |request| < 2 -> exit + re-burst
  # Large-angle holdback - the one envelope the vendor log does not cover
  # (its max observed angle is 37 deg). Our old >50 deg latches all happened
  # through hesitation mechanics that no longer exist, but with no vendor
  # evidence past 37 deg, stand down there and re-burst below 40.
  STEER_LARGE_ANGLE = 50.        # deg, stand down above this
  STEER_LARGE_ANGLE_REARM = 40.  # deg, re-burst below this
  # The vendor streams Config=3 in EVERY session state - idle included
  # (route 00000037: 16 s of Cfg=3/Act=0/lanes 0/0 standby at boot, and
  # between every session). Lanes 0/0 make it standby, not an armed-silent
  # session; the "Config=3 idle latches" reading came from our early
  # from-scratch frames that missed the SETME_* fields.
  STEER_SESSION_CONFIG = 3
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
