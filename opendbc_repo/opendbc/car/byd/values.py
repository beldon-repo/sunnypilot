from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarDocs, CarParts, SupportType
from opendbc.car.lateral import AngleSteeringLimits


class BydSafetyFlags(IntFlag):
  # experimental 482 camera angle path (Atto 3 style); Song Plus DM-i does not
  # transmit 0x1E2, the default (0) is the 790 LKAS_Output torque path
  ANGLE_STEERING = 1


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
  STEER_SOFTSTART_STEP = 6  # per command; 0 -> full in ~0.67 s at the 50 Hz rate
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
