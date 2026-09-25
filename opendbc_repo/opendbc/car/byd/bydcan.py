def byd_checksum(byte_key, dat):
  # not an actual known crc function, reverse engineered on Atto 3 (bukapilot);
  # TODO(Song Plus DM-i): verify against real vehicle CAN logs
  def byte_crc4_linear_inverse(byte_list):
    return (-1 * sum(byte_list) + 0x9) & 0xF

  second_bytes = [byte & 0xF for byte in dat]
  remainder = sum(second_bytes) >> 4
  second_bytes.append(byte_key >> 4)

  first_bytes = [byte >> 4 for byte in dat]
  first_bytes.append(byte_key & 0xF)

  return (((byte_crc4_linear_inverse(first_bytes) + (-1 * remainder + 5)) << 4) + byte_crc4_linear_inverse(second_bytes)) & 0xFF


def byd_checksum_xor(dat):
  # XOR checksum used on ACC_EPS_STATE (from the open BYD port)
  ret = 0
  for byte in dat:
    ret ^= byte
  return ret


def create_lkas_request(packer, apply_torque, lkas_active, lkas_config, raw_cnt):
  """50 Hz, spoofed ACC_MPC_STATE (790) carrying the LKAS torque request."""
  values = {
    "LeftLaneState": 0,
    "RightLaneState": 0,
    "LKAS_Config": lkas_config,          # 2=LKA when active, 1=ALARM otherwise
    "ReqHandsOnSteeringWheel": 0,
    "LKAS_Output": apply_torque,         # steer torque request
    "LKAS_ReqPrepare": 0,
    "LKAS_Active": 1 if lkas_active else 0,
    "LKAS_State": 2 if lkas_active else 0,
    "TrafficSignRecognition_OnOff": 0,
    "TrafficSignRecognition_Result": -5,
    "LKAS_AlarmType": 0,
    "Counter": raw_cnt,
  }

  dat = packer.make_can_msg("ACC_MPC_STATE", 0, values)[1]
  crc = byd_checksum(0xAF, dat[:-1])
  values["CheckSum"] = crc
  return packer.make_can_msg("ACC_MPC_STATE", 0, values)


def send_buttons(packer, count):
  """Spoof ACC UP_RESETSPEED button press (BTN_AccUpDown_Cmd=3) for auto-resume from standstill."""
  values = {
    "SETME_1": 1,
    "BTN_AccUpDown_Cmd": 3,
    "SETME2_1": 1,
    "Counter": count,
  }

  dat = packer.make_can_msg("PCM_BUTTONS", 0, values)[1]
  crc = byd_checksum(0xAF, dat[:-1])
  values["CheckSum"] = crc
  return packer.make_can_msg("PCM_BUTTONS", 0, values)


def create_fake_eps_state(packer, eps_state_msg, fake_torque, counter):
  """Spoof ACC_EPS_STATE (792) EPS feedback so the ADAS domain does not fault (no DTC).

  Not sent by default; only needed if the real EPS feedback is not visible to the
  ADAS domain with the harness in place."""
  values = {s: eps_state_msg[s] for s in [
    "LKAS_Prepared",
    "CruiseActivated",
    "TorqueFailed",
    "SETME1_0x1",
    "SteerWarning",
    "SteerErrorCode",
    "SETME3_0x1",
    "SETME4_0x3",
    "SETME5_0xFF",
    "SETME6_0xFFF",
  ]}
  values["MainTorque"] = fake_torque
  values["SteerDriverTorque"] = 0
  values["ReportHandsNotOnSteeringWheel"] = 0
  values["Counter"] = counter

  dat = packer.make_can_msg("ACC_EPS_STATE", 0, values)[1]
  values["CheckSum"] = byd_checksum_xor(dat[:-1])
  return packer.make_can_msg("ACC_EPS_STATE", 0, values)


def create_can_steer_command(packer, steer_angle, steer_req, is_standstill, raw_cnt):
  """50 Hz, spoofed STEERING_MODULE_ADAS (482) angle command. Experimental angle path only."""
  set_me_xe = 0xB
  if is_standstill:
    set_me_xe = 0xE

  values = {
    "STEER_REQ": steer_req,
    "STEER_REQ_ACTIVE_LOW": not steer_req,
    "STEER_ANGLE": steer_angle * 1.02,     # desired steer angle
    "SET_ME_X01": 0x1 if steer_req else 0,  # must be 0x1 to steer
    "SET_ME_XE": set_me_xe if steer_req else 0,  # 0xB faults lesser, maybe higher value faults lesser,
                                            # 0xB also seems to have the highest angle limit at high speed
    "COUNTER": raw_cnt,
    "SET_ME_FF": 0xFF,
    "SET_ME_F": 0xF,
    "SET_ME_1_1": 1,
    "SET_ME_1_2": 1,
  }

  dat = packer.make_can_msg("STEERING_MODULE_ADAS", 0, values)[1]
  crc = byd_checksum(0xAF, dat[:-1])
  values["CHECKSUM"] = crc
  return packer.make_can_msg("STEERING_MODULE_ADAS", 0, values)


def create_accel_command(packer, accel, enabled, raw_cnt):
  """ACC_CMD (814) spoof for openpilot longitudinal control. Not in use, needs real vehicle validation."""
  # AccelCmd 0|8 (0.05, -5)
  accel_raw = int(round((min(max(accel, -5.0), 7.75) + 5.0) / 0.05))

  values = {
    "AccelCmd": accel_raw,
    "ComfortBandUpper": 100,
    "ComfortBandLower": 100,
    "JerkUpperLimit": 25,
    "SETME1_0x1": 1,
    "JerkLowerLimit": 80,
    "ResumeFromStandstill": 0,
    "StandstillState": 0,
    "BrakeBehaviour": 0,
    "AccReqNotStandstill": enabled,
    "AccControlActive": enabled,
    "AccOverrideOrStandstill": 0,
    "EspBehaviour": 0,
    "Counter": raw_cnt,
  }

  dat = packer.make_can_msg("ACC_CMD", 0, values)[1]
  crc = byd_checksum(0xAF, dat[:-1])
  values["CheckSum"] = crc
  return packer.make_can_msg("ACC_CMD", 0, values)
