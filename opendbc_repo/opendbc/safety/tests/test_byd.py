#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerPanda


class BydButtonTestBase:
  """PCM_BUTTONS (0x3B0) resume spoofing rules, shared by both steering paths."""

  def test_resume_buttons(self):
    # BTN_AccUpDown_Cmd=3 (UP_RESETSPEED) spoof is only allowed while stationary
    for stationary in (True, False):
      self._rx(self._speed_msg(0. if stationary else 5.))

      for controls_allowed in (True, False):
        self.safety.set_controls_allowed(controls_allowed)

        # all-zero message (no button press) is allowed only while stationary
        self.assertEqual(stationary, self._tx(self.packer.make_can_msg_panda("PCM_BUTTONS", 0, {})))

        # resume press
        values = {"BTN_AccUpDown_Cmd": 3}
        self.assertEqual(stationary, self._tx(self.packer.make_can_msg_panda("PCM_BUTTONS", 0, values)))

        # other buttons are never allowed
        for btn in ("BTN_AccCancel", "BTN_TOGGLE_ACC_OnOff", "BTN_AccDistanceDecrease", "BTN_AccDistanceIncrease"):
          values = {btn: 1}
          self.assertFalse(self._tx(self.packer.make_can_msg_panda("PCM_BUTTONS", 0, values)))

        # down (setspeed) press is never allowed
        values = {"BTN_AccUpDown_Cmd": 1}
        self.assertFalse(self._tx(self.packer.make_can_msg_panda("PCM_BUTTONS", 0, values)))

    self._rx(self._speed_msg(0.))


class TestBydSafetyTorque(BydButtonTestBase, common.PandaCarSafetyTest, common.DriverTorqueSteeringSafetyTest):

  # torque path TX whitelist: OP's own messages + CAN gateway forwarding
  # (powertrain bus 0 -> camera bus 2, camera TX -> bus 0); keep in sync
  # with BYD_TX_MSGS_TORQUE in opendbc/safety/modes/byd.h
  TX_MSGS = [[0x316, 0], [0x3B0, 0],
             [0x32D, 0], [0x32E, 0], [0x32F, 0], [0x432, 0],
             [0x055, 2], [0x08C, 2], [0x0D5, 2], [0x10D, 2], [0x10E, 2], [0x11F, 2],
             [0x121, 2], [0x122, 2], [0x123, 2], [0x12C, 2], [0x12D, 2], [0x133, 2],
             [0x151, 2], [0x164, 2], [0x173, 2], [0x1C2, 2], [0x1F0, 2], [0x20A, 2],
             [0x20D, 2], [0x20F, 2], [0x218, 2], [0x219, 2], [0x220, 2], [0x222, 2],
             [0x223, 2], [0x23F, 2], [0x240, 2], [0x241, 2], [0x242, 2], [0x24C, 2],
             [0x251, 2], [0x275, 2], [0x27E, 2], [0x294, 2], [0x2A9, 2], [0x2B6, 2],
             [0x2BF, 2], [0x2D4, 2], [0x2EC, 2], [0x30D, 2], [0x312, 2], [0x318, 2],
             [0x31D, 2], [0x31E, 2], [0x320, 2], [0x321, 2], [0x322, 2], [0x323, 2],
             [0x32C, 2], [0x33B, 2], [0x33C, 2], [0x33D, 2], [0x341, 2], [0x342, 2],
             [0x343, 2], [0x344, 2], [0x34F, 2], [0x356, 2], [0x35C, 2], [0x35F, 2],
             [0x36E, 2], [0x36F, 2], [0x38A, 2], [0x3AC, 2], [0x3AD, 2], [0x3B0, 2],
             [0x3B7, 2], [0x3C5, 2], [0x3CD, 2], [0x3D9, 2], [0x3EC, 2], [0x3FC, 2],
             [0x3FF, 2], [0x404, 2], [0x407, 2], [0x40D, 2], [0x40E, 2], [0x410, 2],
             [0x418, 2], [0x41A, 2], [0x41C, 2], [0x422, 2], [0x434, 2], [0x449, 2],
             [0x44A, 2], [0x475, 2], [0x48B, 2], [0x49A, 2], [0x4A5, 2], [0x4A9, 2],
             [0x4BB, 2], [0x4BF, 2], [0x4D9, 2], [0x4DE, 2], [0x4F9, 2], [0x4FA, 2],
             [0x4FE, 2], [0x511, 2], [0x512, 2], [0x527, 2], [0x52A, 2], [0x539, 2],
             [0x53A, 2]]
  RELAY_MALFUNCTION_ADDRS = {}
  FWD_BLACKLISTED_ADDRS = {}

  GAS_PRESSED_THRESHOLD = 1  # factor 0.01 percent

  # torque control limits (matches BYD_TORQUE_STEERING_LIMITS in byd.h)
  MAX_RATE_UP = 10
  MAX_RATE_DOWN = 12
  MAX_RT_DELTA = 250
  MAX_TORQUE_LOOKUP = ([0.], [300])
  DYNAMIC_MAX_TORQUE = False
  # our actuation bit cut does not lock the mode out; mismatch (torque with
  # LKAS_Active=0) is still blocked, asserted in test_torque_req_mismatch
  NO_STEER_REQ_BIT = True

  DRIVER_TORQUE_ALLOWANCE = 68
  DRIVER_TORQUE_FACTOR = 3

  def setUp(self):
    self.packer = CANPackerPanda("byd_general_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.byd, 0)
    self.safety.init_tests()

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_Output": torque, "LKAS_Active": steer_req}
    return self.packer.make_can_msg_panda("ACC_MPC_STATE", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"SteerDriverTorque": torque}
    return self.packer.make_can_msg_panda("ACC_EPS_STATE", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"AccState": 1 if enable else 0}  # Song: 1 = ACC_ACTIVE, 0/7 = off/standby
    return self.packer.make_can_msg_panda("ACC_HUD_ADAS", 2, values)  # camera side is bus 2

  def _speed_msg(self, speed):
    values = {"WHEELSPEED_BL": speed * 3.6, "WHEELSPEED_BR": speed * 3.6}
    return self.packer.make_can_msg_panda("WHEEL_SPEED", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BrakePressed": brake}
    return self.packer.make_can_msg_panda("DRIVE_STATE", 0, values)

  def _user_gas_msg(self, gas):
    values = {"AcceleratorPedal": gas}
    return self.packer.make_can_msg_panda("PEDAL", 0, values)

  def test_torque_req_mismatch(self):
    # torque request with LKAS_Active=0 is blocked (no tolerance on mismatch)
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(self.MAX_TORQUE)
    self.assertTrue(self._tx(self._torque_cmd_msg(self.MAX_TORQUE, 1)))
    self.assertFalse(self._tx(self._torque_cmd_msg(self.MAX_TORQUE, 0)))


class TestBydSafetyAngle(BydButtonTestBase, common.PandaCarSafetyTest, common.AngleSteeringSafetyTest):
  """Experimental 482 angle path (safetyParam ANGLE_STEERING)."""

  TX_MSGS = [[0x1E2, 0], [0x3B0, 0]]  # STEERING_MODULE_ADAS, PCM_BUTTONS
  RELAY_MALFUNCTION_ADDRS = {0: (0x1E2,)}
  FWD_BLACKLISTED_ADDRS = {2: [0x1E2]}

  GAS_PRESSED_THRESHOLD = 1

  # Angle control limits
  STEER_ANGLE_MAX = 90  # deg, DiPilot faults above this
  DEG_TO_CAN = 10

  ANGLE_RATE_BP = [0., 5., 15.]
  ANGLE_RATE_UP = [3., 1.2, 0.35]   # windup limit
  ANGLE_RATE_DOWN = [3., 2.5, 0.6]  # unwind limit

  def setUp(self):
    self.packer = CANPackerPanda("byd_general_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.byd, 1)  # BydSafetyFlags.ANGLE_STEERING
    self.safety.init_tests()

  def _angle_cmd_msg(self, angle: float, enabled: bool):
    values = {"STEER_ANGLE": angle, "STEER_REQ": 1 if enabled else 0}
    return self.packer.make_can_msg_panda("STEERING_MODULE_ADAS", 0, values)

  def _angle_meas_msg(self, angle: float):
    values = {"SteeringAngle": angle}
    return self.packer.make_can_msg_panda("EPS", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"AccState": 1 if enable else 0}  # Song: 1 = ACC_ACTIVE, 0/7 = off/standby
    return self.packer.make_can_msg_panda("ACC_HUD_ADAS", 2, values)  # camera side is bus 2

  def _speed_msg(self, speed):
    values = {"WHEELSPEED_BL": speed * 3.6, "WHEELSPEED_BR": speed * 3.6}
    return self.packer.make_can_msg_panda("WHEEL_SPEED", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BrakePressed": brake}
    return self.packer.make_can_msg_panda("DRIVE_STATE", 0, values)

  def _user_gas_msg(self, gas):
    values = {"AcceleratorPedal": gas}
    return self.packer.make_can_msg_panda("PEDAL", 0, values)


if __name__ == "__main__":
  unittest.main()
