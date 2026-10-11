"""BYD Song Plus DM-i radar interface - HUD-synthesized single lead.

Why this is not a target-list parser (docs/byd-lane-change.md §8.5, mined
2026-10-11 over the 415-segment archive):

  * 0x374 RADAR_MRR: absent from every recorded bus.
  * 0x351/0x354 (the IDs the vendor's decompiled radar_interface.py branches
    parse for other BYD models): constant-payload idle heartbeats on bus 1,
    AUC 0.5 between lead/no-lead regimes - the Song does not route targets
    through them, so the vendor branch is deliberately not copied here.
  * 0x380-0x38f: a 16-slot @15Hz block that initially looked like a target
    list (per-slot regime "separation" up to AUC 0.86), but a multi-bar
    ordinal test kills it: no byte/word is rank-consistent against the
    0x32D distance bar across segments; the frames are a rolling-counter
    status/telemetry table (shared same-tick byte1 across slots, per-slot
    constants), not objects.

What remains is the stock radar's own fused summary, already plaintext and
validated: 0x32D ACC_HUD_ADAS (bus 2, 50 Hz) HasLead + LeadingDistance
(0-7 gap bars). This interface turns it into one RadarPoint so radarState.
leadOne has a real source. vRel/aRel are NaN (the vendor has no target speed
either, per its decompiled interface, and a 3-bit bar cannot support
differentiation without large sawtooth artifacts) - radard's lead Kalman
filter on dRel is the intended consumer.

Status: inert by default. interface.py keeps CP.radarUnavailable=True until
RADAR_HUD_BARS_TO_M is calibrated on-road; the moment that table is trusted,
flipping the flag lights this up with no other change.
"""
import math

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.byd.values import DBC, RADAR_HUD_BARS_TO_M
from opendbc.car.interfaces import RadarInterfaceBase

ACC_HUD_ADAS_MSG = 813  # 0x32D, bus 2


def _create_radar_can_parser(CP):
  return CANParser(DBC[CP.carFingerprint][Bus.radar], [(ACC_HUD_ADAS_MSG, 50)], 2)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    self.rcp = None if CP.radarUnavailable else _create_radar_can_parser(CP)
    self.updated_messages: set[int] = set()
    self.trigger_msg = ACC_HUD_ADAS_MSG

  def update(self, can_packets):
    if self.rcp is None:
      return super().update(can_packets)

    # same (can_ptr, packets) shape card.py hands CarState; flatten for CANParser
    changed = self.rcp.update(can_packets)
    self.updated_messages.update(changed)
    if self.trigger_msg not in self.updated_messages:
      return None
    self.updated_messages.clear()

    ret = structs.RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True
      return ret

    vl = self.rcp.vl[ACC_HUD_ADAS_MSG]
    if vl["HasLead"] and vl["LeadingDistance"] in RADAR_HUD_BARS_TO_M:
      if 1 not in self.pts:
        self.pts[1] = structs.RadarData.RadarPoint()
        self.pts[1].trackId = 1
      self.pts[1].dRel = RADAR_HUD_BARS_TO_M[int(vl["LeadingDistance"])]
      self.pts[1].yRel = 0.0
      self.pts[1].vRel = float('nan')
      self.pts[1].aRel = float('nan')
      self.pts[1].yvRel = float('nan')
      self.pts[1].measured = True
    else:
      self.pts.pop(1, None)

    ret.points = list(self.pts.values())
    return ret
