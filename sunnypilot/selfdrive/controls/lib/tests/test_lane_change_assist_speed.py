"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Turn-signal lane change assist threshold (LaneChangeAssistSpeed param port,
docs/byd-lane-change.md §三): >0 replaces the stock 20 MPH gate, 0 disables the
assist state machine at any speed (assist-less mode then owns the maneuver).

BYD rear-traffic hardening (docs/byd-lane-change.md §八): BSD 0x418 is the only
rear traffic source on the platform (no radar objects on any logged bus), so a
change may only start after the same-side blindspot has stayed continuously
clear for LANE_CHANGE_CLEAR_TIME_MIN in preLaneChange, and a same-side blindspot
lighting up mid-maneuver aborts the change back to preLaneChange.
"""
from cereal import car, log

from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import (
  DesireHelper, LaneChangeState, LANE_CHANGE_CLEAR_TIME_MIN)
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode

LaneChangeDirection = log.LaneChangeDirection

V_LC = 30 * CV.MPH_TO_MS  # above every tested threshold
FRAMES_CLEAR = int(round(LANE_CHANGE_CLEAR_TIME_MIN / DT_MDL))


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


def feed(dh: DesireHelper, v_ego: float, left_blinker: bool = False, right_blinker: bool = False,
         torque: int = 0, pressed: bool = False, left_bs: bool = False, right_bs: bool = False):
  cs = car.CarState.new_message()
  cs.vEgo = v_ego
  cs.leftBlinker = left_blinker
  cs.rightBlinker = right_blinker
  cs.leftBlindspot = left_bs
  cs.rightBlindspot = right_bs
  cs.steeringTorque = torque
  cs.steeringPressed = pressed
  dh.update(cs, True, 0.0)
  return dh


def start_left_change(dh: DesireHelper, nudge: bool = True) -> DesireHelper:
  """blinker edge + the full clear window, returns dh in laneChangeStarting"""
  dh = feed(dh, V_LC, left_blinker=True)
  assert dh.lane_change_state == LaneChangeState.preLaneChange
  tq = 50 if nudge else 0
  pr = nudge
  for _ in range(FRAMES_CLEAR):
    dh = feed(dh, V_LC, left_blinker=True, torque=tq, pressed=pr)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting
  return dh


class TestLaneChangeAssistSpeed:

  def test_stock_threshold_default_engages_on_blinker_and_nudge(self):
    dh = make_dh(20)
    dh = feed(dh, V_LC, left_blinker=True)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    # clear window has to elapse before the nudge can start the change
    for _ in range(FRAMES_CLEAR - 1):
      dh = feed(dh, V_LC, left_blinker=True, torque=50, pressed=True)
      assert dh.lane_change_state == LaneChangeState.preLaneChange
    dh = feed(dh, V_LC, left_blinker=True, torque=50, pressed=True)
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
    dh = feed(dh, V_LC, left_blinker=True)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    # driver flips the param to 0 mid-maneuver: preLaneChange bails to off
    dh.lane_change_assist_speed_ms = float("inf")
    dh = feed(dh, V_LC, left_blinker=True)
    assert dh.lane_change_state == LaneChangeState.off


class TestBlindspotRearTrafficGate:

  def test_blindspot_at_edge_never_starts(self):
    # stock guarantee preserved: a blindspot hit keeps the change blocked even
    # after the clear window has elapsed
    dh = make_dh(20)
    dh = feed(dh, V_LC, left_blinker=True)
    for _ in range(FRAMES_CLEAR + 5):
      dh = feed(dh, V_LC, left_blinker=True, torque=50, pressed=True, left_bs=True)
      assert dh.lane_change_state == LaneChangeState.preLaneChange

  def test_blindspot_during_window_reopens_it(self):
    dh = make_dh(20)
    dh = feed(dh, V_LC, left_blinker=True)
    for _ in range(FRAMES_CLEAR - 4):  # partial window
      dh = feed(dh, V_LC, left_blinker=True, torque=50, pressed=True)
    dh = feed(dh, V_LC, left_blinker=True, torque=50, pressed=True, left_bs=True)  # resets to 0
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    for _ in range(FRAMES_CLEAR - 1):
      dh = feed(dh, V_LC, left_blinker=True, torque=50, pressed=True)
      assert dh.lane_change_state == LaneChangeState.preLaneChange
    dh = feed(dh, V_LC, left_blinker=True, torque=50, pressed=True)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting

  def test_same_side_blindspot_mid_maneuver_aborts(self):
    dh = start_left_change(make_dh(20))
    dh = feed(dh, V_LC, left_blinker=True, torque=50, pressed=True, left_bs=True)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    assert dh.desire == log.Desire.none  # lat request released
    assert dh.lane_change_direction == LaneChangeDirection.left  # blocked alert keeps the side

  def test_opposite_blindspot_does_not_abort(self):
    dh = start_left_change(make_dh(20))
    dh = feed(dh, V_LC, left_blinker=True, right_bs=True)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting
    assert dh.desire == log.Desire.laneChangeLeft

  def test_abort_requires_full_window_to_restart(self):
    dh = start_left_change(make_dh(20))
    dh = feed(dh, V_LC, left_blinker=True, torque=50, pressed=True, left_bs=True)  # abort
    dh = feed(dh, V_LC, left_blinker=True, torque=50, pressed=True)  # bs gone, window from 0
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    for _ in range(FRAMES_CLEAR - 2):
      dh = feed(dh, V_LC, left_blinker=True, torque=50, pressed=True)
      assert dh.lane_change_state == LaneChangeState.preLaneChange
    dh = feed(dh, V_LC, left_blinker=True, torque=50, pressed=True)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting

  def test_consecutive_change_requires_new_window(self):
    dh = start_left_change(make_dh(20))
    # run starting -> finishing -> preLaneChange (blinker stays on)
    for _ in range(FRAMES_CLEAR):  # ll_prob fades to 0 in .5s -> finishing
      dh = feed(dh, V_LC, left_blinker=True)
    assert dh.lane_change_state == LaneChangeState.laneChangeFinishing
    for _ in range(FRAMES_CLEAR * 2 + 2):  # ll_prob fades in over 1s -> preLaneChange
      dh = feed(dh, V_LC, left_blinker=True)
      if dh.lane_change_state == LaneChangeState.preLaneChange:
        break
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    # an immediate nudge must NOT start the next change - window reopened
    dh = feed(dh, V_LC, left_blinker=True, torque=50, pressed=True)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
