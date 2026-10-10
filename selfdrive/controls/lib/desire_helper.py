from cereal import log, custom
from openpilot.common.constants import CV
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot.common.raw_params import get_int_param
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeController, AutoLaneChangeMode
from openpilot.sunnypilot.selfdrive.controls.lib.lane_turn_desire import LaneTurnController

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

# Default turn-signal lane change speed threshold, overridable at runtime by the
# LaneChangeAssistSpeed param (MPH, vendor dp_lat_lane_change_assist_speed port,
# docs/byd-lane-change.md §三). 0 disables the assist state machine entirely -
# the assist-less yield mode (sunnypilot assist_less_lane_change) then owns the
# blinker+steer-input case by releasing CC.latActive for a manual lane change.
LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.

# BYD hardening (docs/byd-lane-change.md §八): BSD_RADAR 0x418 is the only rear
# traffic input on this platform - no radar objects are ever logged (0x374 has
# zero frames in all 415 road-test segments) - and it refreshes at 10 Hz, so the
# stock one-frame blindspot sample at the starting edge is lag-prone. The stalk
# also decodes a 0.2-0.3 s mechanical transient as a blinker, letting a change
# start before any fresh BSD frame could arrive. Require the same-side blindspot
# to stay continuously clear this long in preLaneChange before starting.
LANE_CHANGE_CLEAR_TIME_MIN = 0.5

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.none,
    LaneChangeState.laneChangeFinishing: log.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeRight,
  },
}

TURN_DESIRES = {
  custom.TurnDirection.none: log.Desire.none,
  custom.TurnDirection.turnLeft: log.Desire.turnLeft,
  custom.TurnDirection.turnRight: log.Desire.turnRight,
}


class DesireHelper:
  def __init__(self):
    self.lane_change_state = LaneChangeState.off
    self.lane_change_direction = LaneChangeDirection.none
    self.lane_change_timer = 0.0
    self.lane_change_ll_prob = 1.0
    self.lane_change_clear_time = 0.0
    self.keep_pulse_timer = 0.0
    self.prev_one_blinker = False
    self.desire = log.Desire.none
    self.alc = AutoLaneChangeController(self)
    self.lane_turn_controller = LaneTurnController(self)
    self.lane_turn_direction = custom.TurnDirection.none
    self.params = Params()
    self.param_read_counter = 0
    self.lane_change_assist_speed_ms = LANE_CHANGE_SPEED_MIN
    self.read_params()

  def read_params(self) -> None:
    # the device's prebuilt params_pyx.so (v0.10.1) has no LaneChangeAssistSpeed
    # registry entry; get_int_param falls back to the raw /data/params file so the
    # threshold stays configurable until the .so is rebuilt by the release CI
    speed_mph = get_int_param(self.params, "LaneChangeAssistSpeed", int(LANE_CHANGE_SPEED_MIN / CV.MPH_TO_MS))
    # inf on 0: every frame reads as "below threshold" so the assist state machine
    # stays off at any speed (vendor's disable intent; their un-decrypted DesireHelper
    # wrapper semantics can't be inspected, docs/byd-lane-change.md §三)
    self.lane_change_assist_speed_ms = speed_mph * CV.MPH_TO_MS if speed_mph > 0 else float("inf")

  def update_params(self) -> None:
    if self.param_read_counter % 50 == 0:
      self.read_params()
    self.param_read_counter += 1

  def update(self, carstate, lateral_active, lane_change_prob):
    self.alc.update_params()
    self.lane_turn_controller.update_params()
    self.update_params()
    v_ego = carstate.vEgo
    one_blinker = carstate.leftBlinker != carstate.rightBlinker
    below_lane_change_speed = v_ego < self.lane_change_assist_speed_ms

    # Lane turn controller update
    self.lane_turn_controller.update_lane_turn(blindspot_left=carstate.leftBlindspot, blindspot_right=carstate.rightBlindspot,
                                               left_blinker=carstate.leftBlinker, right_blinker=carstate.rightBlinker, v_ego=v_ego)
    self.lane_turn_direction = self.lane_turn_controller.get_turn_direction()

    if not lateral_active or self.lane_change_timer > LANE_CHANGE_TIME_MAX or self.alc.lane_change_set_timer == AutoLaneChangeMode.OFF:
      self.lane_change_state = LaneChangeState.off
      self.lane_change_direction = LaneChangeDirection.none
    else:
      # LaneChangeState.off
      if self.lane_change_state == LaneChangeState.off and one_blinker and not self.prev_one_blinker and not below_lane_change_speed:
        self.lane_change_state = LaneChangeState.preLaneChange
        self.lane_change_ll_prob = 1.0
        self.lane_change_clear_time = 0.0

      # LaneChangeState.preLaneChange
      elif self.lane_change_state == LaneChangeState.preLaneChange:
        # Set lane change direction
        self.lane_change_direction = LaneChangeDirection.left if \
          carstate.leftBlinker else LaneChangeDirection.right

        torque_applied = carstate.steeringPressed and \
                         ((carstate.steeringTorque > 0 and self.lane_change_direction == LaneChangeDirection.left) or
                          (carstate.steeringTorque < 0 and self.lane_change_direction == LaneChangeDirection.right))

        blindspot_detected = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                              (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

        self.alc.update_lane_change(blindspot_detected, carstate.brakePressed)

        # rear-traffic clear window (LANE_CHANGE_CLEAR_TIME_MIN): a same-side
        # blindspot hit at any point reopens it, so starting only ever happens
        # behind a continuous run of clear BSD frames
        if blindspot_detected:
          self.lane_change_clear_time = 0.0
        else:
          self.lane_change_clear_time += DT_MDL

        if not one_blinker or below_lane_change_speed:
          self.lane_change_state = LaneChangeState.off
          self.lane_change_direction = LaneChangeDirection.none
        elif (torque_applied or self.alc.auto_lane_change_allowed) and \
                self.lane_change_clear_time >= LANE_CHANGE_CLEAR_TIME_MIN - 1e-6:  # float-sum tolerance
          self.lane_change_state = LaneChangeState.laneChangeStarting
          self.lane_change_clear_time = 0.0

      # LaneChangeState.laneChangeStarting
      elif self.lane_change_state == LaneChangeState.laneChangeStarting:
        # BYD hardening: bail mid-maneuver if the same-side blindspot lights up
        # (BSD is the only rear-traffic source here - once starting the stock
        # machine was unprotected). Returning to preLaneChange zeroes the desire
        # (lat centers back) and makes selfdrived raise laneChangeBlocked; the
        # clear window reopens so the change only restarts once the car has
        # passed. laneChangeFinishing is NOT aborted - swinging back across the
        # line this late is worse than completing the maneuver.
        blindspot_now = ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
                         (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))
        if blindspot_now:
          self.lane_change_state = LaneChangeState.preLaneChange
          self.lane_change_clear_time = 0.0
        else:
          # fade out over .5s
          self.lane_change_ll_prob = max(self.lane_change_ll_prob - 2 * DT_MDL, 0.0)

          # 98% certainty
          if lane_change_prob < 0.02 and self.lane_change_ll_prob < 0.01:
            self.lane_change_state = LaneChangeState.laneChangeFinishing

      # LaneChangeState.laneChangeFinishing
      elif self.lane_change_state == LaneChangeState.laneChangeFinishing:
        # fade in laneline over 1s
        self.lane_change_ll_prob = min(self.lane_change_ll_prob + DT_MDL, 1.0)

        if self.lane_change_ll_prob > 0.99:
          self.lane_change_direction = LaneChangeDirection.none
          if one_blinker:
            # consecutive change (stock "keep blinker on" semantics): re-check
            # the rear before allowing the next one in
            self.lane_change_state = LaneChangeState.preLaneChange
            self.lane_change_clear_time = 0.0
          else:
            self.lane_change_state = LaneChangeState.off

    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.preLaneChange):
      self.lane_change_timer = 0.0
    else:
      self.lane_change_timer += DT_MDL

    self.prev_one_blinker = one_blinker

    if self.lane_turn_direction != custom.TurnDirection.none:
      self.desire = TURN_DESIRES[self.lane_turn_direction]
    else:
      self.desire = DESIRES[self.lane_change_direction][self.lane_change_state]

    # Send keep pulse once per second during LaneChangeStart.preLaneChange
    if self.lane_change_state in (LaneChangeState.off, LaneChangeState.laneChangeStarting):
      self.keep_pulse_timer = 0.0
    elif self.lane_change_state == LaneChangeState.preLaneChange:
      self.keep_pulse_timer += DT_MDL
      if self.keep_pulse_timer > 1.0:
        self.keep_pulse_timer = 0.0
      elif self.desire in (log.Desire.keepLeft, log.Desire.keepRight):
        self.desire = log.Desire.none

    self.alc.update_state()
