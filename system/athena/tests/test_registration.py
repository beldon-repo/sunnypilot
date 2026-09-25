from pathlib import Path

from openpilot.common.params import Params
from openpilot.system.athena.registration import register, UNREGISTERED_DONGLE_ID
from openpilot.system.hardware.hw import Paths


class TestRegistration:

  def setup_method(self):
    # clear params and setup dongle id path
    self.params = Params()

    persist_dir = Path(Paths.persist_root()) / "comma"
    persist_dir.mkdir(parents=True, exist_ok=True)
    self.dongle_id = persist_dir / "dongle_id"

  def test_valid_cache(self):
    # if the dongle id is written, return the cached dongle id without any
    # network calls. should work with a dongle ID on either /persist/ or params
    dongle = "DONGLE_ID_123"
    for persist, params in [(True, True), (True, False), (False, True)]:
      self.params.put("DongleId", dongle if params else "")
      with open(self.dongle_id, "w") as f:
        f.write(dongle if persist else "")
      assert register() == dongle

  def test_no_dongle_id(self):
    # no dongle id anywhere -> fall back to the unregistered placeholder
    dongle = register()
    assert dongle == UNREGISTERED_DONGLE_ID
    assert self.params.get("DongleId") == dongle
