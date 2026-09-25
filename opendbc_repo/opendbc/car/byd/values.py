from dataclasses import dataclass, field

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarDocs, CarParts, SupportType
from opendbc.car.lateral import AngleSteeringLimits


class CarControllerParams:
  # DiPilot faults when the steering request exceeds ~90 degrees,
  # and the angle command is sent at 0.1 deg/bit
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    90.,  # deg, DiPilot faults above this  # TODO(Song Plus DM-i): calibrate from real vehicle data
    ([0., 5., 15.], [3., 1.2, 0.35]),
    ([0., 5., 15.], [3., 2.5, 0.6]),
  )

  STEER_THRESHOLD = 6.0  # eps torque threshold to consider steering pressed

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
  centerToFrontRatio: float = 0.44
  steerRatio: float = 15.  # TODO(Song Plus DM-i): calibrate from real vehicle data


@dataclass
class BYDPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'byd_general_pt'})


class CAR(Platforms):
  BYD_SONG_PLUS_DMI = BYDPlatformConfig(
    [BYDCarDocs("BYD Song Plus DM-i 2021-24")],
    BYDCarSpecs(mass=1885, wheelbase=2.76),
  )


DBC = CAR.create_dbc_map()

# ratio between the wheel speed sensor and the real speed; calibrate from logs
HUD_MULTIPLIER = 1.0
