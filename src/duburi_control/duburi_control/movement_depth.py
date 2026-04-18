#!/usr/bin/env python3
"""
Depth hold loop — extracted from movement_commands.py for symmetry with
the per-axis layout. Behaviour is byte-for-byte identical to the previous
set_depth body: Python PID on Ch3 (throttle) while ArduSub is in ALT_HOLD.

The PID instance is owned by the caller (MovementCommands) so integral
state can persist across calls if we ever want it; today each call still
starts with a fresh reset via stop() upstream.
"""

import time

from .movement_pids  import DepthPID

_HZ_DEPTH  = 10.0
_DEPTH_TOL = 0.10   # metres — ArduSub ALT_HOLD can't resolve tighter
_LOG_EVERY = 0.5


def depth_hold(api, pid: DepthPID,
               target_m: float, timeout: float, logger) -> None:
    """
    Drive the sub to `target_m` using the supplied PID. Caller is expected
    to have already put ArduSub in ALT_HOLD; we do not switch modes here.
    """
    deadline = time.time() + timeout
    prev_t   = time.time()
    closest  = None
    last_log = time.time()

    while time.time() < deadline:
        now    = time.time()
        dt     = max(now - prev_t, 1e-3)
        prev_t = now

        att = api.get_attitude()
        if att is None:
            time.sleep(0.1)
            continue

        current = att['depth']
        error   = abs(target_m - current)
        if closest is None or error < abs(target_m - closest):
            closest = current

        depth_pwm = pid.compute_pwm(target_m, current, dt)
        api.send_rc_override(throttle=depth_pwm)

        if now - last_log >= _LOG_EVERY:
            logger.info(
                f'[DEPTH] → {target_m:.2f}m  '
                f'now:{current:+.2f}m  err:{error:.2f}m  pwm:{depth_pwm}')
            last_log = now

        if error < _DEPTH_TOL:
            logger.info(f'[DEPTH] ✓ {current:+.2f}m')
            return

        time.sleep(1.0 / _HZ_DEPTH)

    if closest is not None:
        logger.info(f'[DEPTH] ⚠ timeout  closest:{closest:+.2f}m')
    else:
        logger.info('[DEPTH] ⚠ timeout — no telemetry')
