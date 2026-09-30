import pytest

from collections import defaultdict

from cereal import custom

from opendbc.car import Bus, structs
from opendbc.car.byd.carstate import BOOT_LATCH_HOLD_TIME, CarState
from opendbc.car.byd.values import CAR


class FakeParser:
  """CANParser stand-in: all signals read 0 unless set. CarState only touches
  .vl[message][signal]."""

  def __init__(self):
    self.vl = defaultdict(lambda: defaultdict(float))


def make_carstate() -> CarState:
  CP = structs.CarParams.new_message()
  CP.carFingerprint = CAR.BYD_SONG_PLUS_DMI_22
  CP_SP = custom.CarParamsSP.new_message()
  return CarState(CP, CP_SP)


def acc_engaged(cs: CarState):
  """Make the raw latch True: radar commanding ACC + HUD AccOn1."""
  cs  # parsers live on the test instance
  return cs


def run_update(cs: CarState, acc_on: bool = True, brake: bool = False):
  pt, adas = FakeParser(), FakeParser()
  if acc_on:
    adas.vl['ACC_CMD']['AccControlActive'] = 1
    adas.vl['ACC_HUD_ADAS']['AccOn1'] = 1
  if brake:
    pt.vl['DRIVE_STATE']['BrakePressed'] = 1
  ret, _ = cs.update({Bus.pt: pt, Bus.adas: adas})
  return ret


HOLD_FRAMES = int(BOOT_LATCH_HOLD_TIME * 100)


class TestBydBootLatchHold:
  """Boot-mid-cruise: the stock ACC is engaged when the device boots.

  cruiseState.enabled must stay False through the boot hold and then produce
  ONE genuine rising edge once the hold expires (vendor mechanism, route
  7--12e) - reporting the raw latch immediately eats the edge in the generic
  layer (canValid / ready window NO_ENTRY) and OP never engages.
  """

  def test_hold_delays_the_engage_edge(self):
    cs = make_carstate()
    for _ in range(10):
      ret = run_update(cs, acc_on=True)
      assert not ret.cruiseState.enabled        # raw latch held low
      assert ret.cruiseState.available          # available stays raw

    cs.boot_frames = HOLD_FRAMES - 2            # last held frame (+1 in update)
    ret = run_update(cs, acc_on=True)
    assert not ret.cruiseState.enabled
    ret = run_update(cs, acc_on=True)           # hold expires -> the one edge
    assert ret.cruiseState.enabled
    ret = run_update(cs, acc_on=True)           # stays engaged, no flapping
    assert ret.cruiseState.enabled

  def test_cancel_during_hold_means_no_edge(self):
    cs = make_carstate()
    for _ in range(10):
      ret = run_update(cs, acc_on=True, brake=True)
      assert not ret.cruiseState.enabled
    cs.boot_frames = HOLD_FRAMES - 1
    ret = run_update(cs, acc_on=True, brake=True)
    assert not ret.cruiseState.enabled          # raw latch is False: held or not
    ret = run_update(cs, acc_on=True)           # ACC re-engaged after the hold
    assert ret.cruiseState.enabled              # -> genuine edge fires then

  def test_no_acc_no_edge(self):
    cs = make_carstate()
    cs.boot_frames = HOLD_FRAMES + 100
    for _ in range(5):
      ret = run_update(cs, acc_on=False)
      assert not ret.cruiseState.enabled


if __name__ == '__main__':
  pytest.main([__file__])
