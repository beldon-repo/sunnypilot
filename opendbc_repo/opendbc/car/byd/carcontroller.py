import math

import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits, apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.byd import bydcan
from opendbc.car.byd.values import USE_ANGLE_STEERING, CarControllerParams

LongCtrlState = structs.CarControl.Actuators.LongControlState

RES_INTERVAL = 125   # frames between resume presses (100 Hz loop)
SNG_WAIT = 310       # frames to wait before first resume press
RES_LEN = 3          # number of resume presses

BRAKE_INHIBIT_PRESSED_THRESHOLD = 3   # frames of brake press before inhibiting lateral
BRAKE_INHIBIT_RELEASE_THRESHOLD = 5   # frames of brake release before re-enabling


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.packer = CANPacker(dbc_names[Bus.pt])

    # lateral state (vendor session architecture)
    self.apply_torque_last = 0
    self.apply_angle_last = 0
    self.lkas_active = False
    self.lkas_brake_inhibit = False
    self.brake_pressed_counter = 0
    self.brake_release_counter = 0
    self.retry_burst = 0      # remaining ReqPrepare frames of the engage/retry burst
    self.silence_counter = 0  # armed |request|~0 backstop

    # SNG auto-resume state
    self.is_sng_check = False
    self.sng_next_press_frame = 0
    self.resume_counter = 0
    self.lead_valid = False

  def _handle_brake_inhibit(self, CS):
    """Inhibit lateral control while the driver brakes (avoids LKS faults), with debounce."""
    if CS.out.brakePressed:
      self.brake_release_counter = 0
      if not self.lkas_brake_inhibit:
        self.brake_pressed_counter += 1
        if self.brake_pressed_counter >= BRAKE_INHIBIT_PRESSED_THRESHOLD:
          self.lkas_brake_inhibit = True
    else:
      self.brake_pressed_counter = 0
      if self.lkas_brake_inhibit:
        self.brake_release_counter += 1
        if self.brake_release_counter >= BRAKE_INHIBIT_RELEASE_THRESHOLD:
          self.lkas_brake_inhibit = False

  def _update_torque_lateral(self, CC, CS):
    """Default torque path: steer via the LKAS_Output request in ACC_MPC_STATE (790).

    Session architecture is the vendor's, frame-proven on its own drive logs
    (route 00000037, docs_site/op_byd_logs/7--12e_0) - the op_byd build never
    latches the EPS because it never produces a state the EPS rejects:

      idle        Config=3, Active=0, lanes 0/0, request 0 streamed
                  unconditionally (their armed-standby posture)
      engage      3-frame ReqPrepare burst with lanes 2/2, then Active=1
                  ~50 ms later - the EPS ack is NOT waited for (EPS
                  LKAS_Prepared stayed 0 for the vendor's entire drive)
      in-session  request = model demand through the driver limit, full
                  authority: they reach -153 against a +166 driver yank and
                  193 absolute, ~8 sign flips in 27 s (smooth)
      dropout     Active->0 then IMMEDIATELY re-burst and re-activate within
                  60-80 ms (twice in the log, zero EPS complaint)
      disengage   request ramps 52->0 over ~80 ms with lanes still 2/2, lanes
                  drop to 0/0, then Active=0

    Every one of our ten TorqueFailed latches was a state the vendor cannot
    produce: armed-silence from yield-to-zero hesitation, an oscillating
    capped request stream (follow-flip kept the lateral loop open), or a
    dithering cold arm. So there is no defense stack here - full-authority
    output, instant exit-and-retry, and the EPS's own declarations
    (TorqueFailed / SteerErrorCode) as the only hard stops.
    """
    if self.frame % 2 != 0:
      return None

    drv = CS.out.steeringTorque
    demand = int(round(CC.actuators.torque * CarControllerParams.STEER_MAX))
    lkas_req_prepare = 0

    # hard stops - declared by the driver or the EPS itself. The vendor has
    # no others: it arms at zero request, fights heavy driver torque, and
    # steers parking maneuvers at 37+ deg and 1-17 km/h.
    allow = CC.latActive and not self.lkas_brake_inhibit and not CS.out.standstill \
      and not CS.torque_failed and not CS.steer_error

    if self.lkas_active:
      if CS.torque_failed:
        # EPS-declared dead: hard cut, no ramp out of an EPS error (it has
        # already latched until ignition-off; there is nothing to be graceful
        # toward)
        new_torque = 0
        self.apply_torque_last = 0
        self.lkas_active = False
        self.silence_counter = 0
      elif allow:
        # armed-silence backstop (see values.py): exit to the retry burst
        # before the measured 0.48 s latch band when the demand loop itself
        # has gone quiet
        if abs(self.apply_torque_last) < 2 and abs(demand) < 6:
          self.silence_counter += 1
          if self.silence_counter >= CarControllerParams.STEER_ZERO_EXIT_FRAMES:
            allow = False
        else:
          self.silence_counter = 0
      if allow:
        new_torque = demand
      else:
        # stand-down: ramp the request to zero (vendor exit ramps 52->0 over
        # ~80 ms), keep Active=1 until it lands, then disarm. If OP is still
        # engaged and the EPS has not declared a fault, re-burst immediately
        # - the vendor re-arms a dropped session within 60-80 ms.
        new_torque = 0
        if self.apply_torque_last == 0:
          self.lkas_active = False
          self.silence_counter = 0
          if CC.latActive and not CS.torque_failed and not CS.steer_error:
            self.retry_burst = 3
    else:
      if allow:
        # engage/retry burst: 3 frames of ReqPrepare with lanes 2/2, request
        # 0, then activate. No ack wait - the burst itself is the handshake.
        lkas_req_prepare = 1
        if self.retry_burst == 0:
          self.retry_burst = 3
        self.retry_burst -= 1
        if self.retry_burst == 0:
          self.lkas_active = True
      new_torque = 0

    self.apply_torque_last = apply_driver_steer_torque_limits(
      new_torque, self.apply_torque_last, drv, CarControllerParams)

    # 50 Hz; echo the stock camera's ACC_MPC_STATE so the SETME_* / MPC_State
    # fields match what the DiPilot ADAS domain expects (from-scratch frames
    # fault the camera). The session Config is 3 in EVERY state - idle
    # included - because that is the vendor's own idle posture (lanes 0/0
    # make it standby, not an armed-silent session).
    if CS.cam_lkas:
      return bydcan.create_lkas_request(
        self.packer, CS.cam_lkas, self.apply_torque_last, self.lkas_active,
        lkas_req_prepare, CarControllerParams.STEER_SESSION_CONFIG, (self.frame // 2) % 16)
    return None

  def _update_angle_lateral(self, CC, CS):
    """Experimental angle path: steer via the STEERING_MODULE_ADAS (482) angle request."""
    apply_angle = apply_std_steer_angle_limits(CC.actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                               CS.out.steeringAngleDeg, CC.latActive, CarControllerParams.ANGLE_LIMITS)

    # engage logic tied to the stock LKA button state
    if CS.lka_on:
      self.lkas_active = True
    if not CS.lka_on and not CS.out.cruiseState.enabled:
      self.lkas_active = False

    if CS.out.steeringTorqueEps > 15:
      apply_angle = CS.out.steeringAngleDeg

    lat_active = CC.latActive and self.lkas_active and not CS.out.standstill

    # 50 Hz
    if self.frame % 2 == 0:
      return bydcan.create_can_steer_command(
        self.packer, apply_angle, lat_active, CS.out.standstill, (self.frame // 2) % 16)
    return None

  def _update_sng_auto_resume(self, CC, CS, can_sends):
    """Spoof the ACC resume button while stationary and a lead is detected."""
    auto_resume_allowed = CC.enabled and CS.out.cruiseState.standstill

    if not auto_resume_allowed:
      self.is_sng_check = False
    else:
      self.lead_valid = CC.hudControl.leadVisible and self.lead_valid

      if not self.is_sng_check:
        self.is_sng_check = True
        self.lead_valid = True
        self.sng_next_press_frame = self.frame + SNG_WAIT
        self.resume_counter = 0

      elif self.resume_counter >= RES_LEN or CS.out.gasPressed or CS.res_btn_pressed:
        self.sng_next_press_frame = max(self.sng_next_press_frame, self.frame + RES_INTERVAL)
        self.resume_counter = 0

      elif self.lead_valid and self.frame > self.sng_next_press_frame:
        can_sends.append(bydcan.send_buttons(self.packer, (CS.counter_pcm_buttons + 1) % 16))
        self.resume_counter += 1

  def _update_longitudinal(self, CC, CS, can_sends):
    """OP longitudinal: transparent replacement of the radar's ACC frames on bus 0.

    Mirrors the vendor build's TX (route 00000037): all three messages at 50 Hz
    unconditionally - ACC_CMD is the radar frame with the acceleration fields
    overridden while engaged, ACC_HUD/ACC_AEB are pure echoes. The stock radar
    keeps owning the session on bus 2, so cruiseState and cancel semantics are
    untouched; the firmware blocks the stock frames bus2->bus0 while the
    LONGITUDINAL safety flag is set."""
    if self.frame % 2 == 0:
      raw_cnt = (self.frame // 2) % 16
      # resume pulse while long control is starting (standstill -> go)
      resume = CC.actuators.longControlState == LongCtrlState.starting
      can_sends.append(bydcan.create_accel_command(
        self.packer, CC.actuators.accel, CC.enabled, CC.longActive, resume,
        CS.radar_acc_msg, raw_cnt))
      if CS.adas_msg:
        can_sends.append(bydcan.create_acc_hud_command(self.packer, CS.adas_msg, raw_cnt))
      if CS.aeb_msg:
        can_sends.append(bydcan.create_acc_aeb_command(self.packer, CS.aeb_msg, raw_cnt))

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    self._handle_brake_inhibit(CS)

    # Transmit the spoofed 0x316 at 50 Hz UNCONDITIONALLY (engaged or not).
    # The camera's own 0x316 is blocked from relaying bus2->bus0 by the
    # firmware fwd hook, so OP is the EPS's only 0x316 source - if we stop
    # transmitting while disengaged, the EPS LKAS subsystem starves and
    # faults the ADAS domain ('check multifunction video controller', EPS
    # SteerWarning latched in real-vehicle logs). Idle frames echo the
    # camera's fields with LKAS_Active=0 / torque 0. The old claim that
    # continuous TX faults the domain only applied to the pre-echo
    # from-scratch frames (missing SETME_* fields). Matches the
    # community-verified BYD_Files controller, which transmits every cycle
    # regardless of engagement.
    steer_send = None
    new_actuators = CC.actuators.as_builder()
    if USE_ANGLE_STEERING:
      if CC.enabled or CC.latActive:
        steer_send = self._update_angle_lateral(CC, CS)
        if steer_send is not None:
          new_actuators.steeringAngleDeg = self.apply_angle_last
    else:
      steer_send = self._update_torque_lateral(CC, CS)
      new_actuators.torque = self.apply_torque_last / CarControllerParams.STEER_MAX
      new_actuators.torqueOutputCan = float(self.apply_torque_last)

    if steer_send is not None:
      can_sends.append(steer_send)

    if self.CP.openpilotLongitudinalControl:
      self._update_longitudinal(CC, CS, can_sends)

    self._update_sng_auto_resume(CC, CS, can_sends)

    self.frame += 1
    return new_actuators, can_sends
