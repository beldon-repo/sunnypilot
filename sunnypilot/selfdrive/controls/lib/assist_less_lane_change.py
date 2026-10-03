"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Assist-less lane change - vendor port (op_byd "rick - assist-less lane change",
controlsd.py:691-703; design docs/byd-lane-change.md §四).

When LaneChangeAssistSpeed is 0 (turn-signal lane change assist disabled), a
blinker plus the driver holding the wheel in the same direction releases
lateral control entirely for the duration of the lane change: the driver steers
manually and openpilot never fights the EPS LKAS session mid-maneuver. The
state latches until BOTH blinkers are off. Sign convention note: BYD
SteerDriverTorque was measured left=positive on archived routes
(docs/byd-lane-change.md §四.1), same as openpilot, so the vendor's
torque-direction test is ported verbatim.
"""
from cereal import car

from openpilot.common.params import Params


class AssistLessLaneChange:
  def __init__(self):
    self.params = Params()

    self.assist_disabled = False
    self.active = False

    self.read_params()

  def read_params(self) -> None:
    try:
      self.assist_disabled = int(self.params.get("LaneChangeAssistSpeed", return_default=True)) == 0
    except (TypeError, ValueError):
      # prebuilt params lib without the key: mode stays off, same as stock
      self.assist_disabled = False

  def update(self, CS: car.CarState) -> bool:
    if not self.assist_disabled:
      # assisted mode owns the blinker+steer case via the desire_helper state machine
      self.active = False
      return False

    # de-activate
    if not CS.leftBlinker and not CS.rightBlinker:
      self.active = False

    # activate: wheel held in the blinker direction (left = positive torque)
    if not self.active and CS.steeringPressed and \
       ((CS.steeringTorque > 0 and CS.leftBlinker) or
        (CS.steeringTorque < 0 and CS.rightBlinker)):
      self.active = True

    return self.active
