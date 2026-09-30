import sys
import types

import pytest

# cereal.messaging / openpilot.common.params_pyx pull arm-only cython .so
# files; stub them so these host-side unit tests can import the module chain.
# Only names referenced at import time are provided and none are exercised on
# the code path under test.
try:
  import cereal.messaging  # noqa: F401
except ImportError:
  _fake = types.ModuleType('cereal.messaging')
  for _name in ('SubMaster', 'PubMaster', 'new_message', 'log_from_bytes',
                'toggle_fake_events', 'set_fake_prefix', 'delete_fake_events',
                'recv_one', 'context'):
    setattr(_fake, _name, lambda *a, **k: None)
  sys.modules['cereal.messaging'] = _fake
try:
  import openpilot.common.params_pyx  # noqa: F401
except ImportError:
  _fake = types.ModuleType('openpilot.common.params_pyx')
  for _name in ('Params', 'ParamKeyFlag', 'ParamKeyType', 'UnknownKeyName'):
    setattr(_fake, _name, type(_name, (), {}))
  sys.modules['openpilot.common.params_pyx'] = _fake
try:
  import openpilot.common.transformations.transformations  # noqa: F401
except ImportError:
  _fake = types.ModuleType('openpilot.common.transformations.transformations')
  for _name in ('ecef_euler_from_ned_single', 'euler2quat_single', 'euler2rot_single',
                'ned_euler_from_ecef_single', 'quat2euler_single', 'quat2rot_single',
                'rot2euler_single', 'rot2quat_single'):
    setattr(_fake, _name, lambda *a, **k: None)
  sys.modules['openpilot.common.transformations.transformations'] = _fake

from cereal import car, log
from opendbc.car import DT_CTRL, structs

from openpilot.selfdrive.car.car_specific import BOOT_ENGAGE_STEADY_TIME, CarSpecificEvents

EventName = log.OnroadEvent.EventName
GearShifter = structs.CarState.GearShifter


def make_byd_cp() -> structs.CarParams:
  CP = structs.CarParams.new_message()
  CP.brand = 'byd'
  CP.pcmCruise = True
  return CP


def make_cs(acc_on: bool, brake: bool = False) -> car.CarState:
  CS = car.CarState.new_message()
  CS.cruiseState.enabled = acc_on   # strict AccControlActive-based latch
  CS.cruiseState.available = acc_on
  CS.gearShifter = GearShifter.drive
  CS.vEgo = 10.0
  CS.brakePressed = brake
  return CS


def run_frames(cse: CarSpecificEvents, n: int, acc_on: bool, cc_enabled: bool = False,
               brake_frames: set[int] | None = None) -> list[bool]:
  """Run n carState frames (100 Hz); returns per-frame pcmEnable presence."""
  brake_frames = brake_frames or set()
  CS_prev = make_cs(acc_on=True)  # suppress the genuine-edge path; synth under test
  fired = []
  for i in range(n):
    CS = make_cs(acc_on=acc_on, brake=i in brake_frames)
    CC = car.CarControl.new_message()
    CC.enabled = cc_enabled
    events = cse.update(CS, CS_prev, CC)
    fired.append(EventName.pcmEnable in events.names)
  return fired


class TestBydBootEngage:
  """Boot-mid-cruise: OP boots after the driver already engaged stock ACC.

  The pcmEnable rising edge is lost (fires while canValid is false /
  wrongCarMode NO_ENTRY is up), so the synth must re-issue it once the
  stock ACC has been continuously engaged for BOOT_ENGAGE_STEADY_TIME.
  """

  def test_synthesizes_after_steady_time(self):
    cse = CarSpecificEvents(make_byd_cp())
    steady = int(BOOT_ENGAGE_STEADY_TIME / DT_CTRL)
    fired = run_frames(cse, steady + 100, acc_on=True)

    assert not any(fired[:steady - 1])        # nothing before 3 s of engagement
    assert all(fired[steady - 1:])            # fires on the 300th engaged frame...
    # ...and self-limits once OP enabled (CC.enabled=True from here on)
    fired_after_enable = run_frames(cse, 10, acc_on=True, cc_enabled=True)
    assert not any(fired_after_enable)
    # even if OP later disables, the synth never comes back
    fired_after_disable = run_frames(cse, 400, acc_on=True, cc_enabled=False)
    assert not any(fired_after_disable)

  def test_no_synth_without_acc(self):
    cse = CarSpecificEvents(make_byd_cp())
    fired = run_frames(cse, int(BOOT_ENGAGE_STEADY_TIME / DT_CTRL) + 100, acc_on=False)
    assert not any(fired)

  def test_brake_resets_the_steady_counter(self):
    cse = CarSpecificEvents(make_byd_cp())
    steady = int(BOOT_ENGAGE_STEADY_TIME / DT_CTRL)
    # ACC on for steady time with a brake press in the middle: the counter
    # must reset, so no synth by the original deadline
    fired = run_frames(cse, steady, acc_on=True, brake_frames=set(range(steady // 2, steady // 2 + 50)))
    assert not any(fired)
    # ...and it fires once 3 s of *continuous* engagement have elapsed after
    # the brake release
    fired = run_frames(cse, steady + 1, acc_on=True)
    assert any(fired)

  def test_retry_through_no_entry(self):
    # a NO_ENTRY at fire time (e.g. the ready window) must not consume the
    # synth: it keeps re-issuing pcmEnable until OP actually enables
    cse = CarSpecificEvents(make_byd_cp())
    steady = int(BOOT_ENGAGE_STEADY_TIME / DT_CTRL)
    fired = run_frames(cse, 10 * 100, acc_on=True, cc_enabled=False)  # blocked 10 s
    assert all(fired[steady:])

  def test_genuine_enable_disables_synth_for_good(self):
    # OP enabled via the genuine edge at boot (ready instantly): the synth
    # arms off permanently, a later cancel + re-SET is the driver's business
    cse = CarSpecificEvents(make_byd_cp())
    assert not any(run_frames(cse, 500, acc_on=True, cc_enabled=True))
    # cancel (ACC off), then re-SET: genuine edge fires via create_common_events,
    # synth must stay silent
    CS_prev = make_cs(acc_on=True)
    CS = make_cs(acc_on=False)  # stock ACC off
    CC = car.CarControl.new_message()
    CC.enabled = True
    cse.update(CS, CS_prev, CC)  # counter reset frame
    CS2 = make_cs(acc_on=True)   # re-SET
    CC2 = car.CarControl.new_message()
    CC2.enabled = False
    events = cse.update(CS2, CS, CC2)
    assert EventName.pcmEnable in events.names  # genuine rising edge
    fired = run_frames(cse, 400, acc_on=True, cc_enabled=False)  # if OP didn't enable
    assert not any(fired)                       # synth stays off


if __name__ == '__main__':
  pytest.main([__file__])
