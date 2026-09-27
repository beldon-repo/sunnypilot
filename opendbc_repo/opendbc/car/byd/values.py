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
  # Magnitude probe history: 50 units sustained for 20+ s at 22-25 km/h ran
  # fault-free twice (routes 00000027 seg0, 00000029 seg0, with p95 at the
  # ceiling), so the old "55-78 kills it" reading (route 0000001c) is now
  # attributed to the walking-speed swing, not magnitude - the rate and angle
  # gates carry that protection. 70 is the next probe (+40% over the felt-nothing
  # 50); if it latches, the UI alert shows it and the fallback is 60/50.
  # The firmware safety limits (300/10/12) are upper bounds and stay untouched.
  STEER_MAX = 70
  STEER_STEP = 2            # 50 Hz command rate (100 Hz control loop)
  STEER_DELTA_UP = 4        # per 50 Hz command; firmware safety allows 10
  STEER_DELTA_DOWN = 6      # per 50 Hz command; firmware safety allows 12
  STEER_DRIVER_ALLOWANCE = 68
  STEER_DRIVER_MULTIPLIER = 3
  STEER_DRIVER_FACTOR = 1
  STEER_ERROR_MAX = 50
  STEER_SOFTSTART_STEP = 2  # per command; 0 -> full in ~0.5 s at the 50 Hz rate
  # EPS LKAS operating envelope, real-vehicle fault map (route 0000001c/20/22):
  # TorqueFailed latches on ANY of: |steering angle| beyond ~50 deg (three
  # reproductions at 43-58 deg) or ~55+ units of request (one reproduction at
  # walking speed, small angle). Within |angle| < 21 deg and <= 15 units the
  # request ran 11 s fault-free. Gate the angle with hysteresis and keep the
  # torque ceiling under the magnitude fault line.
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
  # specs from an independent open BYD port (Song Plus DM-i 2021-23 share these)
  centerToFrontRatio: float = 0.44
  steerRatio: float = 15.  # TODO(Song Plus DM-i): calibrate from real vehicle data


@dataclass
class BYDPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'byd_general_pt'})


class CAR(Platforms):
  BYD_SONG_PLUS_DMI_22 = BYDPlatformConfig(
    [BYDCarDocs("BYD Song Plus DM-i 2022")],
    BYDCarSpecs(mass=1785, wheelbase=2.765),
  )


DBC = CAR.create_dbc_map()

# ratio between the wheel speed sensor and the real speed; calibrate from logs
HUD_MULTIPLIER = 1.0
