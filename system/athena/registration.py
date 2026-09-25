#!/usr/bin/env python3
from pathlib import Path

from openpilot.common.params import Params
from openpilot.selfdrive.selfdrived.alertmanager import set_offroad_alert
from openpilot.system.hardware import PC
from openpilot.system.hardware.hw import Paths


UNREGISTERED_DONGLE_ID = "UnregisteredDevice"

def is_registered_device() -> bool:
  dongle = Params().get("DongleId")
  return dongle not in (None, UNREGISTERED_DONGLE_ID)


def register(show_spinner=False) -> str | None:
  """
  Offline registration: never talks to the comma backend.

  The dongle ID comes from params, or from /persist/comma/dongle_id
  on devices that shipped with one. Devices with neither get an
  unregistered placeholder so openpilot still boots offline.
  """
  params = Params()

  dongle_id: str | None = params.get("DongleId")
  if dongle_id is None and Path(Paths.persist_root()+"/comma/dongle_id").is_file():
    # devices built since early comma 3X production (2/28/24) store it in /persist/
    with open(Paths.persist_root()+"/comma/dongle_id") as f:
      dongle_id = f.read().strip() or None

  if dongle_id is None:
    dongle_id = UNREGISTERED_DONGLE_ID

  params.put("DongleId", dongle_id)
  set_offroad_alert("Offroad_UnregisteredHardware", (dongle_id == UNREGISTERED_DONGLE_ID) and not PC)
  return dongle_id


if __name__ == "__main__":
  print(register())
