"""Host tests for the BYD T8 forward-gap monitor and its lane-change gate
(docs/byd-radar-010x.md §④e/§④f; design in opendbc/car/byd/radar_gap.py).

Layer 1: RadarGapMonitor boundary-cross logic in u-free r=b7/vEgo space.
Layer 2: CarState.update_radar_gap - real CANParser decode of BO_ 265/266.
Layer 3: DesireHelper gates - level 0 is byte-identical to pre-T8 behavior;
         level 1 blocks starting; level 2 aborts only on NEW closing targets.
"""
from types import SimpleNamespace

from cereal import custom
from opendbc.car import CanData, structs
from opendbc.car.byd.carstate import CarState
from opendbc.car.byd.radar_gap import RadarGapMonitor, BAND_BOUNDARY_DOWN, SET_MARGIN, SET_TIME
from opendbc.car.byd.values import CAR
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper, LaneChangeState
from openpilot.sunnypilot.selfdrive.controls.lib.auto_lane_change import AutoLaneChangeMode

V = 20.0                     # test cruise speed, m/s
R = {2: 2.92, 3: 4.79, 4: 6.61}  # plateau medians (monitor header)
DT = 0.01                    # card loop step, matches CarState.update_radar_gap


def step(mon, dt, b7, presence=7, cls=3, v_ego=V, fresh=True):
  mon.update(dt, b7, presence, cls, v_ego, fresh)
  return mon.closing


class TestMonitor:
  def run(self, seq):
    """seq: list of (b7, presence, cls, v_ego) held 1s each -> final closing"""
    mon = RadarGapMonitor()
    for b7, presence, cls, v in seq:
      for _ in range(int(1 / DT)):
        step(mon, DT, b7, presence, cls, v)
    return mon

  def test_stable_follow_never_alerts(self):
    # r wobbles +-6% around the class-3 plateau - inside the false-cross band
    # ④e says a static threshold would trip 29-60% of such frames
    mon = RadarGapMonitor()
    import math
    for k in range(1000):
      b7 = int(R[3] * V * (1 + 0.06 * math.sin(k / 20)))
      assert step(mon, DT, b7) is False
    assert not mon.closing

  def test_sustained_closing_alerts_before_gear_flip(self):
    # linear closure r 4.8 -> 3.2 over ~4 s (0.4 r/s): alert must fire once
    # r sits below boundary*0.88 (=3.39) for 0.4 s with the descent confirmed
    b_mid = BAND_BOUNDARY_DOWN[3]          # 3.855
    mon, alert_r, crossed = RadarGapMonitor(), None, False
    for k in range(1000):
      r = R[3] - 0.4 * k * DT
      step(mon, DT, int(r * V))
      crossed |= (not crossed and r <= b_mid)
      if alert_r is None and mon.closing:
        alert_r = r
        break
    set_lim = b_mid * (1.0 - SET_MARGIN)
    assert alert_r is not None
    assert crossed                      # boundary was reached before the alert
    assert alert_r <= set_lim + 1e-9    # alert is the deep, sustained case
    assert alert_r >= set_lim - 0.4 * (SET_TIME + 0.6)  # ... and not so late it
                                        # trails the flip by more than a debounce
    assert alert_r < b_mid

  def test_debounce_single_dip(self):
    mon = RadarGapMonitor()
    for _ in range(3):
      step(mon, DT, int(3.0 * V))        # far below limit, but only 3 frames
    assert not mon.closing

  def test_release_with_hysteresis(self):
    mon = RadarGapMonitor()
    for k in range(1000):
      r = R[3] - 0.4 * k * DT
      step(mon, DT, int(r * V))
      if mon.closing:
        break
    assert mon.closing
    for _ in range(int(0.5 / DT)):        # 0.5 s back at plateau
      step(mon, DT, int(R[3] * V))
    assert mon.closing                    # hysteresis: not released yet (needs 1 s)
    for _ in range(int(0.5 / DT) + 2):
      step(mon, DT, int(R[3] * V))
    assert not mon.closing

  def test_presence_and_sentinel_filters(self):
    mon = RadarGapMonitor()
    for k in range(1000):
      step(mon, DT, int((R[3] - 0.4 * k * DT) * V), presence=5)
    assert not mon.closing                 # presence nibble != 111 -> invisible (④f)
    mon2 = RadarGapMonitor()
    for k in range(1000):
      step(mon2, DT, 246)                  # range-edge band is never "visible" (④a)
    assert not mon2.visible
    mon3 = RadarGapMonitor()
    for _ in range(100):
      step(mon3, DT, 0)
    assert not mon3.visible                # b7==0 sentinel

  def test_low_speed_not_alerted(self):
    mon = RadarGapMonitor()
    for _ in range(2000):
      step(mon, DT, 2, v_ego=1.0)          # crawling, r huge anyway
    assert not mon.closing

  def test_no_frame_goes_invisible(self):
    mon = RadarGapMonitor()
    for _ in range(1000):
      step(mon, DT, int(3.0 * V), fresh=False)
    assert not mon.visible and not mon.closing

  def test_flat_below_boundary_no_alert_without_descent(self):
    # already parked below the limit (driver chose the tight gap): the slope
    # gate must not keep the clear window shut forever
    mon = RadarGapMonitor()
    for _ in range(2000):
      step(mon, DT, int(3.3 * V))
    assert not mon.closing

  def test_new_target_window(self):
    mon = RadarGapMonitor()
    for _ in range(100):
      step(mon, DT, 0, presence=0)         # invisible >= 0.5 s
    for _ in range(10):
      step(mon, DT, int(R[3] * V))
    assert mon.new_target                  # rising edge latches
    for _ in range(1000):
      step(mon, DT, int(R[3] * V))
    assert not mon.new_target              # expires after the 10 s capture lead


class TestCarStateDecode:
  def make_cs(self):
    CP = structs.CarParams.new_message()
    CP.carFingerprint = CAR.BYD_SONG_PLUS_DMI_22
    CP.radarUnavailable = True             # must not matter: gap_cp is separate
    cs = CarState(CP, custom.CarParamsSP.new_message())
    cs.adas_msg = {"HasLead": 1, "LeadingDistance": 3}
    return cs

  def feed(self, cs, b7, presence_bits, ts):
    d9 = bytearray(8)
    d9[7] = b7
    dA = bytearray(8)
    dA[3] = presence_bits << 5
    return cs.update_radar_gap([(int(ts * 1e9),
                                 [CanData(265, bytes(d9), 1), CanData(266, bytes(dA), 1)])], V)

  def test_decode_and_alert(self):
    cs = self.make_cs()
    for k in range(1000):                  # r 4.8 -> 3.2 at 0.4/s, 25 Hz frames
      r = R[3] - 0.4 * k * DT
      self.feed(cs, int(r * V), 7, k * DT)
    assert cs.gap_mon.closing and getattr(cs.gap_mon, "visible", False)

  def test_no_parser_when_radar_dbc_absent(self):
    CP = structs.CarParams.new_message()
    CP.carFingerprint = CAR.BYD_SONG_PLUS_DMI_22
    cs = CarState(CP, custom.CarParamsSP.new_message())
    assert cs.gap_cp is not None
    assert cs.update_radar_gap([], 20.0) == (cs.gap_mon.closing, cs.gap_mon.new_target)


def cs_state(**kw):
  d = dict(vEgo=25.0, leftBlinker=False, rightBlinker=False, steeringPressed=False,
           steeringTorque=0, leftBlindspot=False, rightBlindspot=False, brakePressed=False,
           radarGapClosing=False, radarGapNew=False)
  d.update(kw)
  return SimpleNamespace(**d)


class TestDesireGate:
  def make_dh(self, gate):
    dh = DesireHelper()
    # the periodic read_params() would pull the device default (0 - the stubbed
    # host params have no BydRadarGapGate) and stomp the manual gate on the
    # very first update; pin it for the duration of the test
    dh.read_params = lambda: None
    dh.radar_gap_gate = gate
    dh.alc.lane_change_set_timer = AutoLaneChangeMode.NUDGE
    return dh

  def to_starting(self, dh, gate_cs=None):
    # left blinker edge -> preLaneChange, then 100 Hz of clear frames with
    # left torque applied until the clear window passes
    dh.update(cs_state(), True, 0.5)
    for _ in range(12):
      dh.update(cs_state(leftBlinker=True, steeringPressed=True, steeringTorque=100), True, 0.5)
    return dh.lane_change_state

  def test_gate0_unchanged_behavior(self):
    dh = self.make_dh(0)
    st = self.to_starting(dh)
    assert st == LaneChangeState.laneChangeStarting

  def test_gate0_ignores_alert_fields(self):
    dh = self.make_dh(0)
    dh.update(cs_state(), True, 0.5)
    for _ in range(20):
      dh.update(cs_state(leftBlinker=True, steeringPressed=True, steeringTorque=100,
                         radarGapClosing=True, radarGapNew=True), True, 0.5)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting

  def test_gate1_blocks_starting(self):
    dh = self.make_dh(1)
    st = self.to_starting(dh)  # helper leaves gate CS default False -> starts?
    assert st == LaneChangeState.laneChangeStarting
    # now force the alert while prepping another change: must never start
    dh2 = self.make_dh(1)
    dh2.update(cs_state(), True, 0.5)
    for _ in range(20):
      dh2.update(cs_state(leftBlinker=True, steeringPressed=True, steeringTorque=100,
                          radarGapClosing=True), True, 0.5)
      assert dh2.lane_change_clear_time == 0.0
    assert dh2.lane_change_state == LaneChangeState.preLaneChange

  def test_gate1_stops_at_alert_midway_even_after_window_met(self):
    dh = self.make_dh(1)
    dh.update(cs_state(), True, 0.5)
    for _ in range(9):   # 0.45 s clear - window not met yet
      dh.update(cs_state(leftBlinker=True, steeringPressed=True, steeringTorque=100), True, 0.5)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    for _ in range(5):   # alert arrives before the window completes
      dh.update(cs_state(leftBlinker=True, steeringPressed=True, steeringTorque=100,
                         radarGapClosing=True), True, 0.5)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    for _ in range(15):  # alert clears, window restarts and completes
      dh.update(cs_state(leftBlinker=True, steeringPressed=True, steeringTorque=100), True, 0.5)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting

  def to_starting_then_alert(self, dh, closing=True, new=True):
    self.to_starting(dh)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting
    for _ in range(3):
      dh.update(cs_state(leftBlinker=True, steeringPressed=True, steeringTorque=100,
                         radarGapClosing=closing, radarGapNew=new), True, 0.5)

  def test_gate2_new_target_aborts(self):
    dh = self.make_dh(2)
    self.to_starting_then_alert(dh)
    assert dh.lane_change_state == LaneChangeState.preLaneChange

  def test_gate2_old_target_does_not_abort(self):
    dh = self.make_dh(2)
    self.to_starting_then_alert(dh, new=False)   # closing, lane-ambiguous (no yRel)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting

  def test_gate1_old_alert_does_not_abort(self):
    dh = self.make_dh(1)
    self.to_starting_then_alert(dh)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting
