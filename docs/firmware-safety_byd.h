#pragma once

// BYD lateral control, ported to the 0.9.x-era panda safety structure for the
// tici-compatible board (official v0.10.1 firmware does not boot on it).
// Same protocol/logic as opendbc/safety/modes/byd.h in the main repo.
//
//  param 0 (default): TORQUE path. The EPS follows the LKAS request carried in
//    ACC_MPC_STATE (0x316): LKAS_Config=LKA, LKAS_Output=steer torque,
//    LKAS_Active=actuation. Stock ACC is used for longitudinal; PCM_BUTTONS
//    (0x3B0) UP_RESETSPEED is spoofed for auto-resume from standstill.
//  param 1: experimental ANGLE path (Atto 3 style), spoofing
//    STEERING_MODULE_ADAS (0x1E2) with a desired angle.
//
// TODO(Song Plus DM-i): verify LKAS_Output scaling, driver torque thresholds
// and the 0x122 wheel speed layout from real vehicle CAN logs.

#define BYD_PARAM_ANGLE_STEERING 1

static bool byd_brake_pedal_pressed = false;
static bool byd_angle_steering = false;

static void byd_rx_hook(const CANPacket_t *to_push) {
  int bus = GET_BUS(to_push);
  int addr = GET_ADDR(to_push);

  if (bus == 0U) {
    // EPS: measured steering angle, factor 0.1 deg, little endian
    if (addr == 0x11FU) {
      int angle_meas_new = to_signed((GET_BYTES(to_push, 0, 2) & 0xFFFFU), 16);
      update_sample(&angle_meas, angle_meas_new);
    }

    // WHEEL_SPEED: rear wheel speeds, factor 0.1 km/h, little endian
    if (addr == 0x122U) {
      uint16_t left_rear = ((GET_BYTE(to_push, 5) << 8) | GET_BYTE(to_push, 4));
      uint16_t right_rear = ((GET_BYTE(to_push, 7) << 8) | GET_BYTE(to_push, 6));
      vehicle_moving = (left_rear | right_rear) != 0U;
      UPDATE_VEHICLE_SPEED(((left_rear + right_rear) / 2.0) * 0.1 / 3.6);
    }

    // PEDAL: analog gas and brake pedal, factor 0.01 (percent)
    if (addr == 0x342U) {
      gas_pressed = GET_BYTE(to_push, 0) > 1U;
      byd_brake_pedal_pressed = GET_BYTE(to_push, 1) > 1U;
    }

    // DRIVE_STATE: brake switch
    if (addr == 0x242U) {
      brake_pressed = GET_BIT(to_push, 37U) || byd_brake_pedal_pressed;
    }

    // ACC_EPS_STATE: EPS feedback to the ADAS domain
    if (addr == 0x318U) {
      int torque_driver_new = to_signed((((GET_BYTE(to_push, 4) & 0xFU) << 8) | GET_BYTE(to_push, 3)), 12);  // SteerDriverTorque 24|12
      update_sample(&torque_driver, torque_driver_new);

      int torque_meas_new = to_signed((((GET_BYTE(to_push, 2) & 0xFU) << 8) | GET_BYTE(to_push, 1)), 12);    // MainTorque 8|12
      update_sample(&torque_meas, torque_meas_new);
    }
  }

  // ACC_HUD_ADAS: stock ACC status
  // NOTE(Song Plus DM-i): the stock DiPilot camera TXes on the camera-side
  // segment (bus 2); it never appears on bus 0.
  if ((addr == 0x32DU) && (bus == 2U)) {
    // AccState 19|3: Song encoding - 0 = OFF, 7 = main on / standby;
    // the engaged value(s) are not confirmed yet, so treat every non-standby
    // state as engaged (pcm_cruise_check only enforces cancellation when the
    // stock ACC turns off).
    uint8_t acc_state = ((GET_BYTE(to_push, 2) >> 3) & 0x7U);
    bool cruise_engaged = (acc_state != 0U) && (acc_state != 7U);
    pcm_cruise_check(cruise_engaged);
  }
}

static bool byd_button_checks(const CANPacket_t *to_send) {
  // PCM_BUTTONS: UP_RESETSPEED spoof (SNG auto-resume) is only allowed while
  // stationary; cancel/toggle/distance buttons are never allowed
  bool violation = false;
  if (GET_ADDR(to_send) == 0x3B0U) {
    // BTN_AccUpDown_Cmd is motorola 4|2: bits 4 (msb) and 3 (lsb)
    uint8_t updown_cmd = ((GET_BYTE(to_send, 0) >> 3) & 0x3U);
    bool other_btns = GET_BIT(to_send, 6U) || GET_BIT(to_send, 8U) || GET_BIT(to_send, 15U) || GET_BIT(to_send, 16U);

    if (other_btns || vehicle_moving || (updown_cmd != 0U && updown_cmd != 3U)) {
      violation = true;
    }
  }
  return violation;
}

static bool byd_tx_hook(const CANPacket_t *to_send) {
  bool tx = true;
  bool violation = false;
  int addr = GET_ADDR(to_send);

  if (byd_angle_steering) {
    const SteeringLimits BYD_ANGLE_STEERING_LIMITS = {
      .max_steer = 900,             // 90 deg, DiPilot faults above this
      .max_rt_interval = 250000,
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
    if (addr == 0x1E2U) {
      // STEER_ANGLE: factor -0.1, little endian, start bit 24
      int desired_angle = to_signed(((GET_BYTE(to_send, 4) << 8) | GET_BYTE(to_send, 3)), 16);
      bool steer_req = GET_BIT(to_send, 21U);

      // no steering control allowed when openpilot is not engaged
      if (steer_req && !controls_allowed) {
        violation = true;
      }

      if (steer_angle_cmd_checks(desired_angle, steer_req, BYD_ANGLE_STEERING_LIMITS)) {
        violation = true;
      }
    }
  } else {
    const SteeringLimits BYD_TORQUE_STEERING_LIMITS = {
      .max_steer = 300,             // matches python STEER_MAX
      .max_rate_up = 10,            // per frame at 50 Hz, matches python STEER_DELTA_UP
      .max_rate_down = 12,          // per frame at 50 Hz, matches python STEER_DELTA_DOWN
      .max_rt_delta = 250,          // 250 ms realtime limit
      .max_rt_interval = 250000,
      .type = TorqueDriverLimited,

      // matches python STEER_DRIVER_ALLOWANCE / STEER_DRIVER_FACTOR
      .driver_torque_allowance = 68,
      .driver_torque_factor = 3,

      .max_torque_error = 350,
    };

    // ACC_MPC_STATE: LKAS torque command (default torque path)
    if (addr == 0x316U) {
      // LKAS_Output 16|11, signed
      int desired_torque = to_signed(((GET_BYTE(to_send, 2) | ((GET_BYTE(to_send, 3) & 0x7U) << 8)) & 0x7FFU), 11);
      bool lka_active = GET_BIT(to_send, 28U);  // LKAS_Active

      // no steering control allowed when openpilot is not engaged
      if (lka_active && !controls_allowed) {
        violation = true;
      }

      if (steer_torque_cmd_checks(desired_torque, lka_active, BYD_TORQUE_STEERING_LIMITS)) {
        violation = true;
      }
    }
  }

  if (byd_button_checks(to_send)) {
    violation = true;
  }

  if (violation) {
    tx = false;
  }

  return tx;
}

static int byd_fwd_hook(int bus_num, int addr) {
  (void)bus_num;
  (void)addr;
  // Song Plus DM-i: the stock camera/radar already sit directly on the
  // powertrain bus through the harness pass-through (proven by the vendor
  // firmware, which does not forward either). Relaying bus0<->bus2 floods
  // both networks with duplicate frames and faults the stock ECUs
  // ('ACC restricted' / 'check multifunction video controller').
  // bus 2 (private ADAS wire) is only tapped for monitoring 0x32D/0x32E.
  return -1;
}

static safety_config byd_init(uint16_t param) {
  byd_angle_steering = GET_FLAG(param, BYD_PARAM_ANGLE_STEERING);

  static const CanMsg BYD_TX_MSGS_TORQUE[] = {
    {0x316, 0, 8},  // ACC_MPC_STATE (LKAS torque request)
    {0x3B0, 0, 8},  // PCM_BUTTONS (SNG auto-resume)
  };

  static const CanMsg BYD_TX_MSGS_ANGLE[] = {
    {0x1E2, 0, 8},  // STEERING_MODULE_ADAS (experimental angle path)
    {0x3B0, 0, 8},  // PCM_BUTTONS (SNG auto-resume)
  };

  static RxCheck byd_rx_checks_torque[] = {
    {.msg = {{0x11F, 0, 5, .frequency = 100U}, { 0 }, { 0 }}},  // EPS
    {.msg = {{0x122, 0, 8, .frequency = 50U}, { 0 }, { 0 }}},   // WHEEL_SPEED
    {.msg = {{0x242, 0, 8, .frequency = 50U}, { 0 }, { 0 }}},   // DRIVE_STATE
    {.msg = {{0x342, 0, 8, .frequency = 50U}, { 0 }, { 0 }}},   // PEDAL
    {.msg = {{0x32D, 2, 8, .frequency = 10U}, { 0 }, { 0 }}},   // ACC_HUD_ADAS (camera side, bus 2)
    {.msg = {{0x318, 0, 8, .frequency = 50U}, { 0 }, { 0 }}},   // ACC_EPS_STATE
  };

  static RxCheck byd_rx_checks_angle[] = {
    {.msg = {{0x11F, 0, 5, .frequency = 100U}, { 0 }, { 0 }}},  // EPS
    {.msg = {{0x122, 0, 8, .frequency = 50U}, { 0 }, { 0 }}},   // WHEEL_SPEED
    {.msg = {{0x242, 0, 8, .frequency = 50U}, { 0 }, { 0 }}},   // DRIVE_STATE
    {.msg = {{0x342, 0, 8, .frequency = 50U}, { 0 }, { 0 }}},   // PEDAL
    {.msg = {{0x32D, 2, 8, .frequency = 10U}, { 0 }, { 0 }}},   // ACC_HUD_ADAS (camera side, bus 2)
  };

  safety_config ret;
  if (GET_FLAG(param, BYD_PARAM_ANGLE_STEERING)) {
    ret = BUILD_SAFETY_CFG(byd_rx_checks_angle, BYD_TX_MSGS_ANGLE);
  } else {
    ret = BUILD_SAFETY_CFG(byd_rx_checks_torque, BYD_TX_MSGS_TORQUE);
  }
  return ret;
}

const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
  .fwd = byd_fwd_hook,
};
