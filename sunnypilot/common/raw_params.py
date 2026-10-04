"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Read helper for params that the device's compiled registry does not know yet.

This fork ships common/params_pyx.so as a prebuilt binary (built by the release
CI, no SConstruct in-tree), and its key registry is compiled from
common/params_keys.h at that moment. Keys added on a dev branch therefore raise
UnknownKeyName from Params.get() on-device until the next release rebuild.

The storage layer itself is just plain files under /data/params/<prefix>/<key>,
reachable through Params.get_param_path() (which does NOT validate the key), so
dev tooling can provision such a param by writing the file directly, e.g.
`echo 25 > /data/params/d/LaneChangeAssistSpeed`. get_int_param() prefers the
registered path and falls back to that raw file, making the param configurable
on-device before the .so catches up; once a rebuilt .so registers the key, the
Params.get() path wins and the fallback goes unused.

Caveats while a key is unregistered: Params.put()/remove()/clearAll() and
sunnylink backup all skip it, so the raw file is never managed automatically.
"""
from openpilot.common.params import Params, UnknownKeyName


def get_int_param(params: Params, key: str, default: int) -> int:
  try:
    return int(params.get(key, return_default=True))
  except UnknownKeyName:
    pass
  except (TypeError, ValueError):
    return default
  try:
    with open(params.get_param_path(key)) as f:
      val = f.read().strip()
    return int(val) if val else default
  except (OSError, ValueError):
    return default
