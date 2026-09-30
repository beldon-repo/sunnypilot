#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerPanda


class BydButtonTestBase:
  """PCM_BUTTONS (0x3B0) resume spoofing rules, shared by both steering paths."""

  # Full relay bus 0 <-> bus 2 (camera and radar are fed through it); block
  # only the LKAS (0x316) and angle (0x1E2) control frames. See byd_fwd_hook.
  FWD_BLACKLISTED_ADDRS = {0: [0x1E2, 0x316], 2: [0x1E2, 0x316]}

  # BYD_ACC_MAIN_FALL_HOLD in byd.h, plus margin
  MAIN_FALL_HOLD_FRAMES = 65

  def _rx_main_off(self):
    """Sustained ACC main-off: past the fall hold, so acc_main_on actually
    drops and the pcm/mads edge machinery sees a genuine main-off."""
    for _ in range(self.MAIN_FALL_HOLD_FRAMES):
      self._rx(self._pcm_status_msg(False))

  # The main fall hold means a SINGLE main-off frame no longer cancels -
  # cruise status keeps reading engaged through the hold. Override the three
  # generic pcm tests to drive a sustained main-off where they assert the drop.
  def test_enable_control_allowed_from_cruise(self):
    self._rx_main_off()
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_disable_control_allowed_from_cruise(self):
    self.safety.set_controls_allowed(1)
    self._rx_main_off()
    self.assertFalse(self.safety.get_controls_allowed())

  def test_cruise_engaged_prev(self):
    for engaged in [True, False]:
      self._rx(self._pcm_status_msg(engaged))
      self.assertEqual(engaged, self.safety.get_cruise_engaged_prev())
      self._rx_main_off()
      self.assertFalse(self.safety.get_cruise_engaged_prev())

  def test_main_survives_short_drop(self):
    # the camera's ~2 s main glitches must not kill actuation permission:
    # a burst of main-off frames shorter than the hold keeps acc_main_on up
    # (hold = 60 frames; the 60th drop frame is the first to let go)
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    for _ in range(59):
      self._rx(self._pcm_status_msg(False))
      self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_standby_posture_is_main_on(self):
    # AccState=1 (standby, also the ignition-leftover posture) + AccOn1=1 is
    # MAIN-ON now: longitudinal actuation permission stays up through brake
    # cancel / CANCEL standby - the session gate lives in the OP controller
    self.safety.set_controls_allowed(False)
    self._rx_main_off()
    self.assertFalse(self.safety.get_controls_allowed())
    values = {"AccState": 1, "AccOn1": 1}
    self._rx(self.packer.make_can_msg_panda("ACC_HUD_ADAS", 2, values))
    self.assertTrue(self.safety.get_controls_allowed())

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

  # torque path TX whitelist: OP's own messages only, matches BYD_TX_MSGS_TORQUE
  # in opendbc/safety/modes/byd.h and the flashed firmware
  TX_MSGS = [[0x316, 0], [0x3B0, 0]]
  # NOTE: 0x316 and 0x3B0 are natively visible on bus 0 on Song Plus DM-i
  # (0x316 via the fingerprint, 0x3B0 from the stalk), so relay malfunction
  # detection cannot be enabled - it would false-trigger and block all control.
  RELAY_MALFUNCTION_ADDRS = {}

  GAS_PRESSED_THRESHOLD = 1  # factor 0.01 percent

  # torque control limits (matches BYD_TORQUE_STEERING_LIMITS in byd.h)
  MAX_RATE_UP = 18
  MAX_RATE_DOWN = 18
  MAX_RT_DELTA = 250
  MAX_TORQUE_LOOKUP = ([0.], [300])
  DYNAMIC_MAX_TORQUE = False
  # our actuation bit cut does not lock the mode out; mismatch (torque with
  # LKAS_Active=0) is still blocked, asserted in test_torque_req_mismatch
  NO_STEER_REQ_BIT = True

  DRIVER_TORQUE_ALLOWANCE = 120
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
    # ACC MAIN posture (route-verified): enable = engaged session frame
    # (AccState=3, AccOn1=1); disable = true main-off (AccState=0, AccOn1=0).
    # AccState=1 + AccOn1=1 would be MAIN-ON standby, NOT an off (it also
    # shows up at ignition and after a brake cancel).
    values = {"AccState": 3 if enable else 0, "AccOn1": 1 if enable else 0}
    return self.packer.make_can_msg_panda("ACC_HUD_ADAS", 2, values)  # camera side is bus 2

  def _speed_msg(self, speed):
    # 0x122 WHEEL_SPEED: used by the standard safety tests for vehicle_moving.
    # NOTE: on Song Plus DM-i the real 0x122 reads all zeros, so the flashed
    # firmware reads speed from 0x1F0 ESP_SPEED (see byd.h); the tests still
    # drive 0x122 here because it is the message the generic test harness expects.
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
  RELAY_MALFUNCTION_ADDRS = {}

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
    # ACC MAIN posture (route-verified): enable = engaged session frame
    # (AccState=3, AccOn1=1); disable = true main-off (AccState=0, AccOn1=0).
    # AccState=1 + AccOn1=1 would be MAIN-ON standby, NOT an off (it also
    # shows up at ignition and after a brake cancel).
    values = {"AccState": 3 if enable else 0, "AccOn1": 1 if enable else 0}
    return self.packer.make_can_msg_panda("ACC_HUD_ADAS", 2, values)  # camera side is bus 2

  def _speed_msg(self, speed):
    # 0x122 WHEEL_SPEED: used by the standard safety tests for vehicle_moving.
    # NOTE: on Song Plus DM-i the real 0x122 reads all zeros, so the flashed
    # firmware reads speed from 0x1F0 ESP_SPEED (see byd.h); the tests still
    # drive 0x122 here because it is the message the generic test harness expects.
    values = {"WHEELSPEED_BL": speed * 3.6, "WHEELSPEED_BR": speed * 3.6}
    return self.packer.make_can_msg_panda("WHEEL_SPEED", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BrakePressed": brake}
    return self.packer.make_can_msg_panda("DRIVE_STATE", 0, values)

  def _user_gas_msg(self, gas):
    values = {"AcceleratorPedal": gas}
    return self.packer.make_can_msg_panda("PEDAL", 0, values)


class TestBydSafetyLong(BydButtonTestBase, common.PandaCarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  """OP longitudinal (safetyParam LONGITUDINAL=2): torque lateral + transparent
  ACC_CMD/ACC_HUD/ACC_AEB replacement on bus 0."""

  # torque lateral + the three ACC-domain frames; matches BYD_TX_MSGS_LONG in byd.h
  TX_MSGS = [[0x316, 0], [0x32D, 0], [0x32E, 0], [0x32F, 0], [0x3B0, 0]]
  # forward block adds the ACC-domain frames (both relay directions)
  FWD_BLACKLISTED_ADDRS = {0: [0x1E2, 0x316, 0x32D, 0x32E, 0x32F],
                           2: [0x1E2, 0x316, 0x32D, 0x32E, 0x32F]}
  RELAY_MALFUNCTION_ADDRS = {}

  GAS_PRESSED_THRESHOLD = 1

  MAX_RATE_UP = 18
  MAX_RATE_DOWN = 18
  MAX_RT_DELTA = 250
  MAX_TORQUE_LOOKUP = ([0.], [300])
  DYNAMIC_MAX_TORQUE = False
  NO_STEER_REQ_BIT = True

  DRIVER_TORQUE_ALLOWANCE = 120
  DRIVER_TORQUE_FACTOR = 3

  # ACC_CMD accel limits, raw units (0.05, -5): [-4.0, 2.0] m/s2
  MAX_ACCEL_RAW = 140
  MIN_ACCEL_RAW = 20
  INACTIVE_ACCEL_RAW = 100

  def setUp(self):
    self.packer = CANPackerPanda("byd_general_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.byd, 2)  # BydSafetyFlags.LONGITUDINAL
    self.safety.init_tests()

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_Output": torque, "LKAS_Active": steer_req}
    return self.packer.make_can_msg_panda("ACC_MPC_STATE", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"SteerDriverTorque": torque}
    return self.packer.make_can_msg_panda("ACC_EPS_STATE", 0, values)

  def _accel_cmd_msg(self, accel_raw, acc_control_active=0):
    # packer takes the physical value (0.05, -5); accel_raw is the CAN raw
    values = {"AccelCmd": accel_raw * 0.05 - 5.0, "AccControlActive": acc_control_active}
    return self.packer.make_can_msg_panda("ACC_CMD", 0, values)

  def _pcm_status_msg(self, enable):
    # ACC MAIN posture (route-verified): enable = engaged session frame
    # (AccState=3, AccOn1=1); disable = true main-off (AccState=0, AccOn1=0).
    # AccState=1 + AccOn1=1 would be MAIN-ON standby, NOT an off (it also
    # shows up at ignition and after a brake cancel).
    values = {"AccState": 3 if enable else 0, "AccOn1": 1 if enable else 0}
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

  def test_accel_cmd_active(self):
    # OP commanding (AccControlActive=1): accel within [-4, 2] m/s2 and
    # controls_allowed required for ANY accel value (the stale-echo guard)
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._accel_cmd_msg(self.INACTIVE_ACCEL_RAW, acc_control_active=1)))
    # bounds
    self.assertTrue(self._tx(self._accel_cmd_msg(self.MAX_ACCEL_RAW, acc_control_active=1)))
    self.assertTrue(self._tx(self._accel_cmd_msg(self.MIN_ACCEL_RAW, acc_control_active=1)))
    # out of bounds
    self.assertFalse(self._tx(self._accel_cmd_msg(self.MAX_ACCEL_RAW + 1, acc_control_active=1)))
    self.assertFalse(self._tx(self._accel_cmd_msg(self.MIN_ACCEL_RAW - 1, acc_control_active=1)))

    # commanding (any accel) without controls_allowed is blocked
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._accel_cmd_msg(self.INACTIVE_ACCEL_RAW, acc_control_active=1)))
    self.assertFalse(self._tx(self._accel_cmd_msg(self.INACTIVE_ACCEL_RAW + 20, acc_control_active=1)))

  def test_accel_cmd_echo_relay(self):
    # AccControlActive=0 (radar relay/standby): accel within limits allowed
    # regardless of controls_allowed - this carries the radar's own braking
    # while OP is disengaged (stock ACC still working)
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._accel_cmd_msg(self.INACTIVE_ACCEL_RAW, acc_control_active=0)))
    self.assertTrue(self._tx(self._accel_cmd_msg(self.MIN_ACCEL_RAW, acc_control_active=0)))
    self.assertTrue(self._tx(self._accel_cmd_msg(self.MAX_ACCEL_RAW, acc_control_active=0)))
    # out of bounds is never allowed
    self.assertFalse(self._tx(self._accel_cmd_msg(self.MIN_ACCEL_RAW - 1, acc_control_active=0)))
    self.assertFalse(self._tx(self._accel_cmd_msg(self.MAX_ACCEL_RAW + 1, acc_control_active=0)))

  def test_torque_req_mismatch(self):
    # torque request with LKAS_Active=0 is blocked (no tolerance on mismatch)
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(self.MAX_TORQUE)
    self.assertTrue(self._tx(self._torque_cmd_msg(self.MAX_TORQUE, 1)))
    self.assertFalse(self._tx(self._torque_cmd_msg(self.MAX_TORQUE, 0)))


if __name__ == "__main__":
  unittest.main()
