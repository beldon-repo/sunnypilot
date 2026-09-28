import math

import numpy as np

from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL
from opendbc.car.lateral import apply_driver_steer_torque_limits, apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.byd import bydcan
from opendbc.car.byd.values import USE_ANGLE_STEERING, CarControllerParams

RES_INTERVAL = 125   # frames between resume presses (100 Hz loop)
SNG_WAIT = 310       # frames to wait before first resume press
RES_LEN = 3          # number of resume presses

BRAKE_INHIBIT_PRESSED_THRESHOLD = 3   # frames of brake press before inhibiting lateral
BRAKE_INHIBIT_RELEASE_THRESHOLD = 5   # frames of brake release before re-enabling


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.packer = CANPacker(dbc_names[Bus.pt])

    # lateral state
    self.apply_torque_last = 0
    self.apply_angle_last = 0
    self.lkas_active = False
    self.steer_softstart_limit = 0
    self.lkas_brake_inhibit = False
    self.brake_pressed_counter = 0
    self.brake_release_counter = 0
    self.angle_gate = False
    self.angle_settle_counter = 0
    self.silence_counter = 0
    self.follow_counter = 0
    self.follow_active = False
    self.last_large_angle_frame = -10 ** 9  # far past: no boot-time lockout

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
    """Default torque path: steer via the LKAS_Output request in ACC_MPC_STATE (790)."""
    # EPS LKAS operating envelope (values.py fault map): requesting outside it
    # latches TorqueFailed until ignition-off, killing lateral for the whole
    # drive. Proven triggers: large |steering angle| (3 reproductions) and any
    # request during a full-lock-speed swing (route 00000027 seg 1: a parking
    # unwind swept the wheel through center at ~150 deg/s, the instantaneous
    # re-arm fired at the 0-crossing, and -48 units into that swing latched
    # the EPS). So: stand down above the angle/rate limits, and re-arm only
    # after the wheel has SETTLED - small angle and low rate, held.
    ang = abs(CS.out.steeringAngleDeg)
    rate = abs(CS.out.steeringRateDeg)
    if ang > CarControllerParams.STEER_LARGE_ANGLE:
      # past the EPS fault line: close the gate and stamp the excursion time
      self.angle_gate = True
      self.angle_settle_counter = 0
      self.last_large_angle_frame = self.frame
    elif ang > CarControllerParams.STEER_ANGLE_GATE_DEACT or rate > CarControllerParams.STEER_RATE_DEACT:
      self.angle_gate = True
      self.angle_settle_counter = 0
    elif self.angle_gate and ang < CarControllerParams.STEER_ANGLE_GATE_REARM and rate < CarControllerParams.STEER_RATE_REARM \
          and (self.frame - self.last_large_angle_frame) * DT_CTRL > CarControllerParams.STEER_LARGE_ANGLE_LOCKOUT:
      self.angle_settle_counter += 1
      if self.angle_settle_counter >= CarControllerParams.STEER_ANGLE_SETTLE_FRAMES:
        self.angle_gate = False
    elif not (ang < CarControllerParams.STEER_ANGLE_GATE_REARM and rate < CarControllerParams.STEER_RATE_REARM):
      self.angle_settle_counter = 0

    lat_active = CC.latActive and not self.lkas_brake_inhibit and not CS.out.standstill \
      and not self.angle_gate

    # A latched EPS TorqueFailed (real drive, route 0000001c seg 0: fault fired
    # ~0.7 s into the torque ramp while the driver resisted, then stayed
    # latched) invalidates everything - stand down to plain idle echo, no
    # LKAS_Active and no ReqPrepare, until the EPS clears it. Never treat a
    # stale LKAS_Prepared as armed across the fault.
    if CS.torque_failed:
      self.lkas_active = False
      self.apply_torque_last = 0  # hard cut on fault - no ramp out of an EPS error

    # engage only when the EPS reports the stock LKA prepared; deactivate
    # only after the request has ramped back to zero (the working vendor build
    # ramps 52->0 over ~80 ms and then drops Active - a request step straight
    # from 50 to 0 is something the stock camera never sends)
    if lat_active and not self.lkas_active and not CS.torque_failed:
      # arm ONLY on real demand (> 6 units), first engage included. Drive 2
      # (hour_logs_2) proved the EPS latches TorqueFailed ~0.5 s into an armed
      # (Config=3/Active=1) session whose request is zero - Re_014, Rf_015,
      # R13_003 and R15_001 all armed straight into a driver fight (ACC re-SET
      # / MADS re-engage while the driver held the wheel) and sat at zero
      # request until the EPS raised SteerWarning and latched. No demand, no
      # session: the idle echo below is the camera's own standby state.
      if abs(CC.actuators.torque) > 0.03:
        if CS.lkas_prepared:
          self.lkas_active = True
          self.steer_softstart_limit = 0
          self.follow_counter = 0
          self.follow_active = False
    elif not lat_active and self.apply_torque_last == 0:
      self.lkas_active = False

    lkas_req_prepare = 0
    if self.lkas_active:
      # compute at the 50 Hz command rate so the per-command delta limits match
      # the firmware safety model exactly (the control loop runs at 100 Hz)
      if self.frame % 2 == 0:
        # Drive 2 (hour_logs_2) pinned the EPS latch rules byte-for-byte, 6/6:
        # an armed (Config=3/Active=1) session latches TorqueFailed when its
        # request is ~zero for ~0.5 s (SteerWarning fires ~0.2 s in first -
        # R10_000/R13_003 latched 0.48-0.64 s after the old yield-to-zero
        # parked the request at 0 mid-assist) or when the request OPPOSES the
        # driver (R16_000: -70 vs +46, under the old 68 allowance). The
        # working vendor build does neither: through entire drv>150 fights its
        # request keeps FOLLOWING the driver - nonzero, same direction.
        # Detect opposition from INTENT (demand), not from the applied output:
        # apply_driver_steer_torque_limits clamps opposing requests to 0 once
        # |drv| > ~135 (68 + STEER_MAX/3), so a post-clip test is blind exactly
        # in the heaviest fights (drive 2 measured drv 111-240) - the request
        # would sit at 0 and re-create the armed-silence latch this guards
        # against. Sustained opposition flips the request to a small
        # same-direction follow; the session never goes silent or opposing
        # while the driver has the wheel.
        drv = CS.out.steeringTorque
        demand = int(round(CC.actuators.torque * CarControllerParams.STEER_MAX))
        opposing = demand * drv < 0 and abs(drv) > CarControllerParams.STEER_DRIVER_OPPOSING
        if opposing:
          self.follow_counter += 2
        elif self.follow_counter > 0:
          self.follow_counter -= 1
        if self.follow_counter >= 6:
          self.follow_active = True
        if self.follow_active and abs(drv) < CarControllerParams.STEER_DRIVER_OPPOSING:
          self.follow_active = False
          self.follow_counter = 0
        # Controller-local deactivations must ramp out: brake inhibit, angle
        # gate and standstill turn lat_active off while CC.actuators.torque
        # still carries model demand (MADS default REMAIN_ACTIVE keeps
        # latActive true under braking), and the driver-limit clip never
        # zeroes a same-direction request. Without this cut the armed session
        # keeps steering through braking and into the >50 deg fault zone. The
        # delta limits give the smooth ramp-out; the apply==0 disarm below
        # then ends the session.
        if not lat_active:
          new_torque = 0
        elif self.follow_active:
          new_torque = int(math.copysign(min(abs(demand), CarControllerParams.STEER_FOLLOW_TORQUE), drv))
        elif opposing:
          new_torque = 0  # detection window (~3 commands): neither fight nor sit silent
        else:
          # vendor yield curve (decrypted op_byd): scale the request down
          # smoothly as the driver's grip grows - the feel source behind the
          # vendor's "request follows the driver, never fights" signature.
          # Same-direction demand is scaled too (a helping hand needs less
          # assist); the direction itself never flips here.
          yield_factor = float(np.interp(abs(drv), CarControllerParams.STEER_YIELD_DRV_BP,
                                         CarControllerParams.STEER_YIELD_FACTOR))
          new_torque = int(round(demand * yield_factor))

        new_torque = min(max(new_torque, -self.steer_softstart_limit, -CarControllerParams.STEER_MAX),
                         self.steer_softstart_limit)
        if self.steer_softstart_limit < CarControllerParams.STEER_MAX:
          self.steer_softstart_limit += CarControllerParams.STEER_SOFTSTART_STEP
        self.apply_torque_last = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last,
                                                                  drv, CarControllerParams)

        # Backstop for genuine zero demand (dead-straight road, hands off): an
        # armed session may sit at |request| < 2 for at most ~0.16 s. The EPS
        # raises SteerWarning ~0.2 s into armed silence and latches
        # TorqueFailed at ~0.5 s (drive 2: every latch came 0.48-0.72 s after
        # the request hit zero, 6/6). Exit returns to the idle echo - the
        # camera's own standby state - and the next real demand re-enters:
        # through the 3-frame prepare burst if the EPS dropped LKAS_Prepared
        # on our exit, else directly (which it does is unverified - check in
        # the road-test logs).
        # While opposition is being detected (or the follow flip is winding
        # down), this guard must NOT count: the follow machinery owns the
        # response there (flip at 0.15 s), and counting concurrently raced the
        # silence exit by a single command frame.
        if opposing or self.follow_counter > 0:
          self.silence_counter = 0
        elif abs(self.apply_torque_last) < 2:
          self.silence_counter += 1
          if self.silence_counter >= CarControllerParams.STEER_SILENCE_FRAMES:
            self.lkas_active = False
            self.apply_torque_last = 0
            self.silence_counter = 0
        else:
          self.silence_counter = 0
    else:
      self.apply_torque_last = 0
      self.silence_counter = 0
      if lat_active and not CS.lkas_prepared and not CS.torque_failed:
        # ask the EPS to arm LKA; it responds with LKAS_Prepared in ACC_EPS_STATE
        lkas_req_prepare = 1

    # 50 Hz; echo the stock camera's ACC_MPC_STATE so the SETME_* / MPC_State
    # fields match what the DiPilot ADAS domain expects (from-scratch frames
    # fault the camera). Skip until we have seen the camera's 0x316.
    if self.frame % 2 == 0 and CS.cam_lkas:
      # LKAS_Config=3 (ALARM_AND_LKA) is the EPS's actuation-permission
      # session mode - but ONLY while the session is actually steering or
      # requesting prepare. An armed session (Config=3, Active=0) that sits
      # at zero torque latches the EPS (route 00000004 seg16 et al.), so
      # disengaged/idle frames echo the camera's own standby Config (1/2)
      # - exactly what the stock camera streams when it is not steering.
      lkas_config = 3 if (self.lkas_active or lkas_req_prepare) else None
      return bydcan.create_lkas_request(
        self.packer, CS.cam_lkas, self.apply_torque_last, self.lkas_active,
        lkas_req_prepare, lkas_config, (self.frame // 2) % 16)
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

    self._update_sng_auto_resume(CC, CS, can_sends)

    self.frame += 1
    return new_actuators, can_sends
