from openpilot.common.params import Params
from openpilot.selfdrive.ui.widgets.ssh_key import ssh_key_item
from openpilot.sunnypilot.common.raw_params import get_float_param, put_raw_param
from openpilot.system.ui.widgets import Widget
from openpilot.system.ui.widgets.list_view import multiple_button_item, toggle_item
from openpilot.system.ui.widgets.scroller import Scroller

# BYD 0x109 b7 distance scale u [m/count] (docs/byd-radar-010x.md ②; pinned 0.25
# 2026-10-19 by owner decision, kept tunable for the road test). The gap monitor
# is u-free (r=b7/vEgo counts space) and the b7 counts path reads the factor-1
# MSG_109_B7 placeholder, so changing this can only affect future meters-side
# consumers (track synthesis / distance display), never control behavior.
RADAR_SCALE_PARAM = "BydRadarDistScale"
RADAR_SCALE_DEFAULT = 0.25
RADAR_SCALE_CHOICES = [0.20, 0.25, 0.3048, 0.30]

# Description constants
DESCRIPTIONS = {
  'enable_adb': (
    "ADB (Android Debug Bridge) allows connecting to your device over USB or over the network. " +
    "See https://docs.comma.ai/how-to/connect-to-comma for more info."
  ),
  'joystick_debug_mode': "Preview the driver facing camera to ensure that driver monitoring has good visibility. (vehicle must be off)",
  'ssh_key': (
    "Warning: This grants SSH access to all public keys in your GitHub settings. Never enter a GitHub username " +
    "other than your own. A comma employee will NEVER ask you to add their GitHub username."
  ),
}


class DeveloperLayout(Widget):
  def __init__(self):
    super().__init__()
    self._params = Params()
    items = [
      toggle_item(
        "Enable ADB",
        description=DESCRIPTIONS["enable_adb"],
        initial_state=self._params.get_bool("AdbEnabled"),
        callback=self._on_enable_adb,
      ),
      ssh_key_item("SSH Key", description=DESCRIPTIONS["ssh_key"]),
      toggle_item(
        "Joystick Debug Mode",
        description=DESCRIPTIONS["joystick_debug_mode"],
        initial_state=self._params.get_bool("JoystickDebugMode"),
        callback=self._on_joystick_debug_mode,
      ),
      toggle_item(
        "Longitudinal Maneuver Mode",
        description="",
        initial_state=self._params.get_bool("LongitudinalManeuverMode"),
        callback=self._on_long_maneuver_mode,
      ),
      toggle_item(
        "openpilot Longitudinal Control (Alpha)",
        description="",
        initial_state=self._params.get_bool("AlphaLongitudinalEnabled"),
        callback=self._on_alpha_long_enabled,
      ),
      multiple_button_item(
        "BYD Radar Distance Scale",
        description=(
          "Distance scale u (meters per radar count) for the BYD 0x109 b7 field. "
          "Pinned at 0.25 m/count; adjust only if a measured stop-to-lead gap "
          "disagrees. Lane-change gap gating is scale-free and unaffected."
        ),
        buttons=[f"{u:g} m" for u in RADAR_SCALE_CHOICES],
        button_width=255,
        callback=self._on_radar_scale,
        selected_index=self._radar_scale_index(),
        icon="speed_limit.png",
      ),
    ]

    self._scroller = Scroller(items, line_separator=True, spacing=0)

  def _render(self, rect):
    self._scroller.render(rect)

  def _on_enable_adb(self): pass
  def _on_joystick_debug_mode(self): pass
  def _on_long_maneuver_mode(self): pass
  def _on_alpha_long_enabled(self): pass

  def _radar_scale_index(self) -> int:
    u = get_float_param(self._params, RADAR_SCALE_PARAM, RADAR_SCALE_DEFAULT)
    return min(range(len(RADAR_SCALE_CHOICES)), key=lambda i: abs(RADAR_SCALE_CHOICES[i] - u))

  def _on_radar_scale(self, button_index: int):
    put_raw_param(self._params, RADAR_SCALE_PARAM, RADAR_SCALE_CHOICES[button_index])
