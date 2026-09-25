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
  STEER_MAX = 300
  STEER_STEP = 2            # 50 Hz command rate (100 Hz control loop)
  STEER_DELTA_UP = 7        # per command at 50 Hz; ISO 11270 jerk limit
  STEER_DELTA_DOWN = 12     # per command at 50 Hz
  STEER_DRIVER_ALLOWANCE = 68
  STEER_DRIVER_MULTIPLIER = 3
  STEER_DRIVER_FACTOR = 1
  STEER_ERROR_MAX = 50
  STEER_SOFTSTART_STEP = 9  # torque ramp-in step during soft start
  STEER_THRESHOLD = 56      # driver torque threshold for steeringPressed (byd2 tuning)  # TODO(Song Plus DM-i): calibrate

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
