#pragma once

#include "opendbc/safety/safety_declarations.h"

// BYD DiPilot camera-based lateral control.
// Port based on the open BYD support in bukapilot (MIT licensed), using the
// byd_general_pt.dbc message set shared across BYD DiPilot vehicles.
//
// The car port spoofs the ADAS camera messages:
//  - STEERING_MODULE_ADAS (0x1E2): desired steering angle, 0.1 deg/bit
//  - LKAS_HUD_ADAS (0x316):       LKA HUD state
//  - PCM_BUTTONS (0x3B0):         SET/RES spoof for auto-resume from standstill
// Stock ACC is used for longitudinal control.

static bool byd_brake_pedal_pressed = false;

static void byd_rx_hook(const CANPacket_t *msg) {

  if (msg->bus == 0U) {
    // STEER_MODULE_2: measured steering angle, factor -0.1, little endian
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
  }

  // ACC_HUD_ADAS: stock ACC status
  if ((msg->addr == 0x32DU) && (msg->bus == 0U)) {
    bool cruise_engaged = GET_BIT(msg, 20U) || GET_BIT(msg, 22U);  // ACC_ON2, ACC_ON1
    pcm_cruise_check(cruise_engaged);
  }
}

static bool byd_tx_hook(const CANPacket_t *msg) {
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
    // TODO(Song Plus DM-i): calibrate limits from real vehicle data
  };

  bool tx = true;
  bool violation = false;

  // STEERING_MODULE_ADAS: steering angle command
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

  // PCM_BUTTONS: only SET/RES spoof for auto-resume from standstill is allowed
  if (msg->addr == 0x3B0U) {
    bool other_btns = GET_BIT(msg, 19U) || GET_BIT(msg, 15U) || GET_BIT(msg, 16U) || GET_BIT(msg, 6U);

    // resume spoofing is only used while stationary (SNG), never while moving
    if (other_btns || vehicle_moving) {
      violation = true;
    }
  }

  if (violation) {
    tx = false;
  }

  return tx;
}

static safety_config byd_init(uint16_t param) {
  static const CanMsg BYD_TX_MSGS[] = {
    {0x1E2, 0, 8, .check_relay = true},   // STEERING_MODULE_ADAS
    {0x316, 0, 8, .check_relay = true},   // LKAS_HUD_ADAS
    {0x3B0, 0, 8, .check_relay = false},  // PCM_BUTTONS (SNG auto-resume)
  };

  static RxCheck byd_rx_checks[] = {
    {.msg = {{0x11F, 0, 5, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // STEER_MODULE_2 (5 bytes on Song Plus DM-i)
    {.msg = {{0x122, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // WHEEL_SPEED
    {.msg = {{0x342, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // PEDAL
    {.msg = {{0x242, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // DRIVE_STATE
    {.msg = {{0x32D, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // ACC_HUD_ADAS
  };

  UNUSED(param);

  safety_config ret;
  SET_TX_MSGS(BYD_TX_MSGS, ret);
  SET_RX_CHECKS(byd_rx_checks, ret);
  return ret;
}

const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
};
