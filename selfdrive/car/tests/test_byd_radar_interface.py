"""Host tests for the BYD HUD-synthesized radar interface (docs §8.5)."""
import pytest

from cereal import custom
from opendbc.car import CanData, structs
from opendbc.car.byd.radar_interface import RadarInterface
from opendbc.car.byd.values import CAR, RADAR_HUD_BARS_TO_M

MSG_T_NS = int(1e9 / 50)  # 50 Hz, matches ACC_HUD_ADAS rate


def hud_data(has_lead: bool, bar: int) -> bytes:
  # dbc layout (byd_general_pt): HasLead = bit 9 (byte1 b1), LeadingDistance =
  # bits 13-15 (byte1 b5-7). A real CANParser decoding these bytes is itself
  # the bit-position regression check.
  dat = bytearray(8)
  dat[1] = ((1 if has_lead else 0) << 1) | (bar << 5)
  return bytes(dat)


def make_cp(radar_unavailable: bool) -> structs.CarParams:
  CP = structs.CarParams.new_message()
  CP.carFingerprint = CAR.BYD_SONG_PLUS_DMI_22
  CP.radarUnavailable = radar_unavailable
  return CP


def make_ri(radar_unavailable: bool) -> RadarInterface:
  return RadarInterface(make_cp(radar_unavailable), custom.CarParamsSP.new_message())


def feed(ri: RadarInterface, has_lead: bool, bar: int, n: int = 60) -> structs.RadarDataT | None:
  out = None
  for i in range(n):
    ret = ri.update([(i * MSG_T_NS, [CanData(0x32D, hud_data(has_lead, bar), 2)])])
    if ret is not None:
      out = ret
  return out


class TestInert:
  def test_no_parser_when_radar_unavailable(self):
    ri = make_ri(True)
    assert ri.rcp is None

  def test_never_produces_tracks_when_unavailable(self):
    ri = make_ri(True)
    out = feed(ri, True, 4)
    # base RadarInterfaceBase returns an empty RadarData every 5 frames
    if out is not None:
      assert len(out.points) == 0


class TestLeadSynthesis:
  def test_lead_present_makes_one_point(self):
    out = feed(make_ri(False), True, 4)
    assert out is not None
    assert len(out.points) == 1
    assert out.points[0].dRel == pytest.approx(RADAR_HUD_BARS_TO_M[4])
    assert out.points[0].trackId == 1
    assert out.points[0].measured
    assert out.points[0].vRel != out.points[0].vRel  # NaN

  def test_bar_ordering_and_switch(self):
    ri = make_ri(False)
    feed(ri, True, 2)
    assert ri.pts[1].dRel == pytest.approx(RADAR_HUD_BARS_TO_M[2])
    feed(ri, True, 4)
    assert ri.pts[1].dRel == pytest.approx(RADAR_HUD_BARS_TO_M[4])  # same track, updated gap

  def test_no_lead_clears_point(self):
    ri = make_ri(False)
    feed(ri, True, 4)
    assert len(ri.pts) == 1
    out = feed(ri, False, 0)
    assert len(ri.pts) == 0
    assert len(out.points) == 0

  def test_all_table_bars_parse(self):
    # every bar in RADAR_HUD_BARS_TO_M must decode back identically through the
    # real CANParser (guards the 13|3@1+ bit placement)
    for bar, dist in RADAR_HUD_BARS_TO_M.items():
      assert bar <= 7
      ri = make_ri(False)
      feed(ri, True, bar)
      assert ri.pts[1].dRel == pytest.approx(dist)
