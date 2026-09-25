#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerPanda


class TestBydSafety(common.PandaCarSafetyTest, common.AngleSteeringSafetyTest):

  TX_MSGS = [[0x1E2, 0], [0x316, 0], [0x3B0, 0]]  # STEERING_MODULE_ADAS, LKAS_HUD_ADAS, PCM_BUTTONS
  RELAY_MALFUNCTION_ADDRS = {0: (0x1E2, 0x316)}
  FWD_BLACKLISTED_ADDRS = {2: [0x1E2, 0x316]}

  GAS_PRESSED_THRESHOLD = 1  # factor 0.01 percent

  # Angle control limits
  STEER_ANGLE_MAX = 90  # deg, DiPilot faults above this
  DEG_TO_CAN = 10

  ANGLE_RATE_BP = [0., 5., 15.]
  ANGLE_RATE_UP = [3., 1.2, 0.35]   # windup limit
  ANGLE_RATE_DOWN = [3., 2.5, 0.6]  # unwind limit

  def setUp(self):
    self.packer = CANPackerPanda("byd_general_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.byd, 0)
    self.safety.init_tests()

  def _angle_cmd_msg(self, angle: float, enabled: bool):
    values = {"STEER_ANGLE": angle, "STEER_REQ": 1 if enabled else 0}
    return self.packer.make_can_msg_panda("STEERING_MODULE_ADAS", 0, values)

  def _angle_meas_msg(self, angle: float):
    values = {"STEER_ANGLE_2": angle}
    return self.packer.make_can_msg_panda("STEER_MODULE_2", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"ACC_ON1": enable, "ACC_ON2": enable}
    return self.packer.make_can_msg_panda("ACC_HUD_ADAS", 0, values)

  def _speed_msg(self, speed):
    values = {"WHEELSPEED_BL": speed * 3.6, "WHEELSPEED_BR": speed * 3.6}
    return self.packer.make_can_msg_panda("WHEEL_SPEED", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BRAKE_PRESSED": brake}
    return self.packer.make_can_msg_panda("DRIVE_STATE", 0, values)

  def _user_gas_msg(self, gas):
    values = {"GAS_PEDAL": gas}
    return self.packer.make_can_msg_panda("PEDAL", 0, values)

  def test_resume_buttons(self):
    # SET/RES spoofing (SNG auto-resume) is only allowed while stationary
    for stationary in (True, False):
      self._rx(self._speed_msg(0. if stationary else 5.))

      for controls_allowed in (True, False):
        self.safety.set_controls_allowed(controls_allowed)

        # all-zero message (no button press) is allowed only while stationary
        self.assertEqual(stationary, self._tx(self.packer.make_can_msg_panda("PCM_BUTTONS", 0, {})))

        # SET/RES press
        values = {"SET_BTN": 1, "RES_BTN": 1}
        tx = self._tx(self.packer.make_can_msg_panda("PCM_BUTTONS", 0, values))
        self.assertEqual(tx, stationary)

        # other buttons are never allowed
        for btn in ("ACC_ON_BTN", "LKAS_ON_BTN", "DEC_DISTANCE_BTN", "INC_DISTANCE_BTN"):
          values = {btn: 1}
          self.assertFalse(self._tx(self.packer.make_can_msg_panda("PCM_BUTTONS", 0, values)))

    self._rx(self._speed_msg(0.))


if __name__ == "__main__":
  unittest.main()
