#pragma once

#include "opendbc/safety/safety_declarations.h"

// BYD lateral control, two paths selected by safetyParam:
//
//  param 0 (default): TORQUE path. The EPS follows the LKAS request carried in
//    ACC_MPC_STATE (0x316): LKAS_Config=LKA, LKAS_Output=steer torque,
//    LKAS_Active=actuation. Stock ACC is used for longitudinal; PCM_BUTTONS
//    (0x3B0) UP_RESETSPEED is spoofed for auto-resume from standstill.
//    Message layouts are from byd_han_dmev_2020.dbc, extracted from a working
//    BYD device.
//
//  param 1: experimental ANGLE path (bukapilot Atto 3 style), spoofing
//    STEERING_MODULE_ADAS (0x1E2) with a desired angle. Song Plus DM-i does
//    not transmit 0x1E2, so this path is kept only as a fallback.
//
// TODO(Song Plus DM-i): verify LKAS_Output scaling, driver torque thresholds
// and the 0x122 wheel speed layout from real vehicle CAN logs.

#define BYD_PARAM_ANGLE_STEERING 1

static bool byd_brake_pedal_pressed = false;

static void byd_rx_hook(const CANPacket_t *msg) {

  if (msg->bus == 0U) {
    // EPS: measured steering angle, factor 0.1 deg, little endian
    if (msg->addr == 0x11FU) {
      int angle_meas_new = to_signed((GET_BYTES(msg, 0, 2) & 0xFFFFU), 16);
      update_sample(&angle_meas, angle_meas_new);
    }

    // WHEEL_SPEED: rear wheel speeds, factor 0.1 km/h, little endian
    if (msg->addr == 0x122U) {
      uint16_t left_rear = ((msg->data[5] << 8) | msg->data[4]);
      uint16_t right_rear = ((msg->data[7] << 8) | msg->data[6]);
      vehicle_moving = (left_rear | right_rear) != 0U;
      UPDATE_VEHICLE_SPEED(((left_rear + right_rear) / 2.0) * 0.1 * KPH_TO_MS);
    }

    // PEDAL: analog gas and brake pedal, factor 0.01 (percent)
    if (msg->addr == 0x342U) {
      gas_pressed = msg->data[0] > 1U;
      byd_brake_pedal_pressed = msg->data[1] > 1U;
    }

    // DRIVE_STATE: brake switch
    if (msg->addr == 0x242U) {
      brake_pressed = GET_BIT(msg, 37U) || byd_brake_pedal_pressed;
    }

    // ACC_EPS_STATE: EPS feedback to the ADAS domain
    if (msg->addr == 0x318U) {
      int torque_driver_new = to_signed((((msg->data[4] & 0xFU) << 8) | msg->data[3]), 12);  // SteerDriverTorque 24|12
      update_sample(&torque_driver, torque_driver_new);

      int torque_meas_new = to_signed((((msg->data[2] & 0xFU) << 8) | msg->data[1]), 12);  // MainTorque 8|12
      update_sample(&torque_meas, torque_meas_new);
    }
  }

  // ACC_HUD_ADAS: stock ACC status
  if ((msg->addr == 0x32DU) && (msg->bus == 0U)) {
    // AccState 19|3: 0=OFF, 2=ACC_ON, 3=ACC_ACTIVE, 5=FORCE_ACCEL, 7=ERROR
    uint8_t acc_state = ((msg->data[2] >> 3) & 0x7U);
    bool cruise_engaged = (acc_state == 3U) || (acc_state == 5U);
    pcm_cruise_check(cruise_engaged);
  }
}

static bool byd_button_checks(const CANPacket_t *msg) {
  // PCM_BUTTONS: UP_RESETSPEED spoof (SNG auto-resume) is only allowed while
  // stationary; cancel/toggle/distance buttons are never allowed
  bool violation = false;
  if (msg->addr == 0x3B0U) {
    // BTN_AccUpDown_Cmd is motorola 4|2: bits 4 (msb) and 3 (lsb)
    uint8_t updown_cmd = ((msg->data[0] >> 3) & 0x3U);
    bool other_btns = GET_BIT(msg, 6U) || GET_BIT(msg, 8U) || GET_BIT(msg, 15U) || GET_BIT(msg, 16U);

    if (other_btns || vehicle_moving || (updown_cmd != 0U && updown_cmd != 3U)) {
      violation = true;
    }
  }
  return violation;
}

static bool byd_tx_hook(const CANPacket_t *msg) {
  bool tx = true;
  bool violation = false;

  if (GET_FLAG(current_safety_param, BYD_PARAM_ANGLE_STEERING)) {
    const AngleSteeringLimits BYD_STEERING_LIMITS = {
      .max_angle = 900,               // 90 deg, DiPilot faults above this
      .angle_deg_to_can = 10,
      .angle_rate_up_lookup = {
        {0., 5., 15.},
        {3., 1.2, 0.35}
      },
      .angle_rate_down_lookup = {
        {0., 5., 15.},
        {3., 2.5, 0.6}
      },
    };

    // STEERING_MODULE_ADAS: steering angle command (experimental angle path)
    if (msg->addr == 0x1E2U) {
      // STEER_ANGLE: factor -0.1, little endian, start bit 24
      int desired_angle = to_signed(((msg->data[4] << 8) | msg->data[3]), 16);
      bool steer_req = GET_BIT(msg, 21U);

      // no steering control allowed when openpilot is not engaged
      if (steer_req && !controls_allowed) {
        violation = true;
      }

      if (steer_angle_cmd_checks(desired_angle, steer_req, BYD_STEERING_LIMITS)) {
        violation = true;
      }
    }
  } else {
    const TorqueSteeringLimits BYD_TORQUE_STEERING_LIMITS = {
      .max_torque = 300,              // matches python STEER_MAX
      .max_rate_up = 10,              // per frame at 50 Hz, matches python STEER_DELTA_UP
      .max_rate_down = 12,            // per frame at 50 Hz, matches python STEER_DELTA_DOWN
      .max_rt_delta = 250,            // 250 ms realtime limit: 25 frames x STEER_DELTA_UP(10) at 50 Hz
      .type = TorqueDriverLimited,

      // matches python STEER_DRIVER_ALLOWANCE / STEER_DRIVER_MULTIPLIER
      .driver_torque_allowance = 68,
      .driver_torque_multiplier = 3,

      .max_torque_error = 350,
    };

    // ACC_MPC_STATE: LKAS torque command (default torque path)
    if (msg->addr == 0x316U) {
      // LKAS_Output 16|11, signed
      int desired_torque = to_signed(((msg->data[2] | ((msg->data[3] & 0x7U) << 8)) & 0x7FFU), 11);
      bool lka_active = GET_BIT(msg, 28U);  // LKAS_Active

      // steer_torque_cmd_checks blocks any nonzero torque while not engaged
      if (steer_torque_cmd_checks(desired_torque, lka_active, BYD_TORQUE_STEERING_LIMITS)) {
        violation = true;
      }
    }
  }

  if (byd_button_checks(msg)) {
    violation = true;
  }

  if (violation) {
    tx = false;
  }

  return tx;
}

static safety_config byd_init(uint16_t param) {
  static const CanMsg BYD_TX_MSGS_TORQUE[] = {
    {0x316, 0, 8, .check_relay = true},   // ACC_MPC_STATE (LKAS torque request)
    {0x3B0, 0, 8, .check_relay = false},  // PCM_BUTTONS (SNG auto-resume)
  };

  static const CanMsg BYD_TX_MSGS_ANGLE[] = {
    {0x1E2, 0, 8, .check_relay = true},   // STEERING_MODULE_ADAS (experimental angle path)
    {0x3B0, 0, 8, .check_relay = false},  // PCM_BUTTONS (SNG auto-resume)
  };

  static RxCheck byd_rx_checks_torque[] = {
    {.msg = {{0x11F, 0, 5, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // EPS
    {.msg = {{0x122, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // WHEEL_SPEED
    {.msg = {{0x242, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // DRIVE_STATE
    {.msg = {{0x342, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // PEDAL
    {.msg = {{0x32D, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // ACC_HUD_ADAS
    {.msg = {{0x318, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // ACC_EPS_STATE
  };

  static RxCheck byd_rx_checks_angle[] = {
    {.msg = {{0x11F, 0, 5, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // EPS
    {.msg = {{0x122, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // WHEEL_SPEED
    {.msg = {{0x242, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // DRIVE_STATE
    {.msg = {{0x342, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // PEDAL
    {.msg = {{0x32D, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // ACC_HUD_ADAS
  };

  safety_config ret;
  if (GET_FLAG(param, BYD_PARAM_ANGLE_STEERING)) {
    SET_TX_MSGS(BYD_TX_MSGS_ANGLE, ret);
    SET_RX_CHECKS(byd_rx_checks_angle, ret);
  } else {
    SET_TX_MSGS(BYD_TX_MSGS_TORQUE, ret);
    SET_RX_CHECKS(byd_rx_checks_torque, ret);
  }
  return ret;
}

const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
};
