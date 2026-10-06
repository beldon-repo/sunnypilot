"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Turn-signal lane change assist threshold (LaneChangeAssistSpeed param port,
docs/byd-lane-change.md §三): >0 replaces the stock 20 MPH gate, 0 disables the
assist state machine at any speed (assist-less mode then owns the maneuver).
"""
from cereal import car, log

from openpilot.common.constants import CV
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, LaneChangeState
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode


def make_dh(threshold_mph: float) -> DesireHelper:
  dh = DesireHelper()
  # freeze param reads (tests must not depend on the machine's param store)
  dh.param_read_counter = 1
  dh.alc.param_read_counter = 1
  dh.lane_turn_controller.param_read_counter = 1
  dh.lane_turn_controller.enabled = False
  dh.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGE  # stock behavior: needs a steer nudge
  dh.alc.auto_lane_change_allowed = False
  dh.lane_change_assist_speed_ms = float("inf") if threshold_mph == 0 else threshold_mph * CV.MPH_TO_MS
  return dh


def feed(dh: DesireHelper, v_ego: float, left_blinker: bool = False, torque: int = 0, pressed: bool = False):
  cs = car.CarState.new_message()
  cs.vEgo = v_ego
  cs.leftBlinker = left_blinker
  cs.steeringTorque = torque
  cs.steeringPressed = pressed
  dh.update(cs, True, 0.0)
  return dh


class TestLaneChangeAssistSpeed:

  def test_stock_threshold_default_engages_on_blinker_and_nudge(self):
    dh = make_dh(20)
    dh = feed(dh, 30 * CV.MPH_TO_MS, left_blinker=True)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    dh = feed(dh, 30 * CV.MPH_TO_MS, left_blinker=True, torque=50, pressed=True)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting
    assert dh.desire == log.Desire.laneChangeLeft

  def test_below_threshold_stays_off(self):
    dh = make_dh(20)
    dh = feed(dh, 10 * CV.MPH_TO_MS, left_blinker=True, torque=50, pressed=True)
    assert dh.lane_change_state == LaneChangeState.off

  def test_raised_threshold_blocks_old_pass_speed(self):
    dh = make_dh(30)
    dh = feed(dh, 25 * CV.MPH_TO_MS, left_blinker=True, torque=50, pressed=True)
    assert dh.lane_change_state == LaneChangeState.off
    dh = make_dh(30)
    dh = feed(dh, 35 * CV.MPH_TO_MS, left_blinker=True)
    assert dh.lane_change_state == LaneChangeState.preLaneChange

  def test_zero_disables_assist_at_any_speed(self):
    dh = make_dh(0)
    for v in (5 * CV.MPH_TO_MS, 60 * CV.MPH_TO_MS):
      dh = feed(dh, v, left_blinker=True, torque=50, pressed=True)
      assert dh.lane_change_state == LaneChangeState.off
      assert dh.desire == log.Desire.none

  def test_zero_aborts_an_in_progress_change(self):
    dh = make_dh(20)
    dh = feed(dh, 30 * CV.MPH_TO_MS, left_blinker=True)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    # driver flips the param to 0 mid-maneuver: preLaneChange bails to off
    dh.lane_change_assist_speed_ms = float("inf")
    dh = feed(dh, 30 * CV.MPH_TO_MS, left_blinker=True)
    assert dh.lane_change_state == LaneChangeState.off
