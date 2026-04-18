#!/usr/bin/env python3
"""
Yaw command implementations — two variants with identical signatures:

  yaw_step(api, start, target, timeout, label, logger)
      Bang-bang. Send final target as a step setpoint at 10 Hz. ArduSub's
      internal 400 Hz attitude stabiliser handles the rotation profile.
      Fast, proven, matches every successful mission log to date.

  yaw_ramp(api, start, target, timeout, label, logger)
      Smoothed. Two phases:
        1. Stream a smootherstep-interpolated setpoint from `start` to
           `target` over `duration` seconds (derived from angle size).
        2. Hold the final target until AHRS reports locked for N frames.
      Slower but no overshoot — the setpoint's own rate decays to zero at
      the target, so the sub cannot accelerate past it.

Both release Ch4 (via send_rc_translation) so SET_ATTITUDE_TARGET keeps
authority, and both use the same tolerance / lock-frame exit criterion
so callers see identical success/timeout semantics.
"""

import time

from .mavlink_api      import MavlinkAPI
from .motion_profiles  import smootherstep

# ─── Shared constants (apply to both variants) ─────────────────────────
_HZ_YAW    = 10.0   # SET_ATTITUDE_TARGET stream rate (ArduSub needs ≥ 1 Hz)
_YAW_TOL   = 2.0    # deg — ArduSub's attitude stabiliser accuracy
_YAW_LOCK  = 5      # consecutive frames within tol before success
_LOG_EVERY = 0.5    # seconds between [YAW] log lines

# ─── Ramp-only tunables ────────────────────────────────────────────────
_YAW_AVG_DPS = 30.0  # average deg/s across a ramped turn
                     # 90° → 3.0s, 45° → 1.5s, 180° → 6.0s
                     # peak rate = avg × 1.875 (smootherstep peak derivative)
_YAW_MIN_DUR = 1.5   # lower bound so small turns still get a ramp


# ---------------------------------------------------------------------- #
#  yaw_step — original bang-bang behaviour                                #
# ---------------------------------------------------------------------- #
def yaw_step(api, start_yaw: float, target_yaw: float,
             timeout: float, label: str, logger) -> None:
    signed = MavlinkAPI.heading_error(target_yaw, start_yaw)
    logger.info(
        f'[CMD  ] yaw_{label.lower()}  {abs(signed):.0f}°  '
        f'cur={start_yaw:.1f}°  tgt={target_yaw:.1f}°  (STEP)')

    api.send_rc_translation()

    deadline = time.time() + timeout
    confirm  = 0
    last_log = time.time()
    peak_err = 0.0
    current  = start_yaw

    while time.time() < deadline:
        now = time.time()

        api.set_attitude_setpoint(yaw_deg=target_yaw)
        api.send_rc_translation()

        att = api.get_attitude()
        if att is None:
            time.sleep(0.1)
            continue

        current = att['yaw']
        err     = MavlinkAPI.heading_error(target_yaw, current)
        peak_err = max(peak_err, abs(err))

        if now - last_log >= _LOG_EVERY:
            logger.info(
                f'[YAW  ] {label}  '
                f'tgt:{target_yaw:.1f}°  cur:{current:.1f}°  err:{err:+.1f}°')
            last_log = now

        if abs(err) <= _YAW_TOL:
            confirm += 1
            if confirm >= _YAW_LOCK:
                logger.info(
                    f'[YAW  ] ✓ {label} locked at {current:.1f}°  '
                    f'(peak err {peak_err:.1f}°)')
                return
        else:
            confirm = 0

        time.sleep(1.0 / _HZ_YAW)

    logger.info(
        f'[YAW  ] ⚠ {label} timeout  cur:{current:.1f}°  tgt:{target_yaw:.1f}°')


# ---------------------------------------------------------------------- #
#  yaw_ramp — smoothed setpoint sweep                                     #
# ---------------------------------------------------------------------- #
def yaw_ramp(api, start_yaw: float, target_yaw: float,
             timeout: float, label: str, logger) -> None:
    signed   = MavlinkAPI.heading_error(target_yaw, start_yaw)
    duration = max(_YAW_MIN_DUR, abs(signed) / _YAW_AVG_DPS)

    logger.info(
        f'[CMD  ] yaw_{label.lower()}  {abs(signed):.0f}°  '
        f'cur={start_yaw:.1f}°  tgt={target_yaw:.1f}°  (RAMP {duration:.1f}s)')

    api.send_rc_translation()

    # ── Phase 1: stream a smootherstep-swept setpoint ──────────────────
    t0       = time.time()
    last_log = t0
    current  = start_yaw

    while True:
        elapsed = time.time() - t0
        if elapsed >= duration:
            break

        frac  = elapsed / duration
        swept = (start_yaw + signed * smootherstep(frac)) % 360

        api.set_attitude_setpoint(yaw_deg=swept)
        api.send_rc_translation()

        att = api.get_attitude()
        if att is not None:
            current = att['yaw']

        now = time.time()
        if now - last_log >= _LOG_EVERY:
            logger.info(
                f'[YAW  ] {label}  sweep:{swept:6.1f}°  cur:{current:6.1f}°  '
                f'({frac*100:3.0f}%)')
            last_log = now

        time.sleep(1.0 / _HZ_YAW)

    # ── Phase 2: lock at the final target ──────────────────────────────
    deadline = time.time() + timeout
    confirm  = 0
    peak_err = 0.0

    while time.time() < deadline:
        api.set_attitude_setpoint(yaw_deg=target_yaw)
        api.send_rc_translation()

        att = api.get_attitude()
        if att is None:
            time.sleep(0.1)
            continue

        current = att['yaw']
        err     = MavlinkAPI.heading_error(target_yaw, current)
        peak_err = max(peak_err, abs(err))

        now = time.time()
        if now - last_log >= _LOG_EVERY:
            logger.info(
                f'[YAW  ] {label}  lock  cur:{current:.1f}°  err:{err:+.1f}°')
            last_log = now

        if abs(err) <= _YAW_TOL:
            confirm += 1
            if confirm >= _YAW_LOCK:
                logger.info(
                    f'[YAW  ] ✓ {label} locked at {current:.1f}°  '
                    f'(peak err {peak_err:.1f}°)')
                return
        else:
            confirm = 0

        time.sleep(1.0 / _HZ_YAW)

    logger.info(
        f'[YAW  ] ⚠ {label} timeout  cur:{current:.1f}°  tgt:{target_yaw:.1f}°')
