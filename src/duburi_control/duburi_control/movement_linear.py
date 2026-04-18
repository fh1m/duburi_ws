#!/usr/bin/env python3
"""
Linear translation commands — two variants with identical signatures:

  linear_step(api, direction, duration, gain, logger)
      Bang-bang. Constant-gain RC override on Ch5/Ch6 for the full
      duration. Exits with FULL velocity, so a short reverse-kick
      brake is applied before the settle.

  linear_ramp(api, direction, duration, gain, logger)
      S-curve. Thrust scale follows trapezoid_ramp:
        smootherstep ease-in → cruise at gain → smootherstep ease-out.
      The ease-out IS the brake — residual velocity is small at exit,
      so no reverse kick is applied, only a settle.

Each variant owns its own exit semantics (brake + settle) so the
facade stays a pure dispatcher and the step vs ramp comparison is
apples-to-apples at the command boundary.
"""

import time

from .mavlink_api      import MavlinkAPI
from .motion_profiles  import trapezoid_ramp

# ─── Shared constants ──────────────────────────────────────────────────
_HZ_LINEAR   = 20.0   # RC override publish rate
_LOG_EVERY   = 0.5    # seconds between [DIR] log lines

# ─── Ramp-only tunable ─────────────────────────────────────────────────
_LINEAR_RAMP = 0.4    # ease-in / ease-out duration (seconds) each side

# ─── Brake / settle tunables (STEP variant only) ───────────────────────
_REV_KICK_PCT = 25    # gentle — higher values push the sub backward
_REV_KICK_SEC = 0.20
_SETTLE_SEC   = 1.2

# Translation gain sign per direction.  (forward-axis, lateral-axis)
_LINEAR_SIGN = {
    'forward': (+1,  0),
    'back':    (-1,  0),
    'left':    ( 0, -1),
    'right':   ( 0, +1),
}

# Which channel/sign to push during the reverse kick.
_BRAKE = {
    'forward': ('forward', -_REV_KICK_PCT),
    'back':    ('forward', +_REV_KICK_PCT),
    'left':    ('lateral', +_REV_KICK_PCT),
    'right':   ('lateral', -_REV_KICK_PCT),
}


# ---------------------------------------------------------------------- #
#  Public variants                                                         #
# ---------------------------------------------------------------------- #
def linear_step(api, direction: str, duration: float,
                gain: int, logger) -> None:
    """Constant gain + aggressive reverse-kick brake — original behaviour."""
    _run_translation(api, direction, duration, gain, logger,
                     scale_fn=lambda _elapsed: 1.0)
    _brake_kick_then_settle(api, direction, logger)


def linear_ramp(api, direction: str, duration: float,
                gain: int, logger) -> None:
    """Smootherstep-shaped ease-in + cruise + ease-out (ease-out is the brake)."""
    _run_translation(api, direction, duration, gain, logger,
                     scale_fn=lambda elapsed:
                         trapezoid_ramp(elapsed, duration, _LINEAR_RAMP))
    _brake_settle_only(api, logger)


# ---------------------------------------------------------------------- #
#  Shared loop                                                             #
# ---------------------------------------------------------------------- #
def _run_translation(api, direction: str, duration: float,
                     gain: int, logger, scale_fn) -> None:
    fwd_sign, lat_sign = _LINEAR_SIGN[direction]

    att        = api.get_attitude()
    locked_hdg = att['yaw'] if att else 0.0
    max_drift  = 0.0
    start      = time.time()
    last_log   = start

    while True:
        now     = time.time()
        elapsed = now - start
        if elapsed >= duration:
            break

        att = api.get_attitude()
        if att is not None:
            drift     = MavlinkAPI.heading_error(locked_hdg, att['yaw'])
            max_drift = max(max_drift, abs(drift))
        else:
            drift = 0.0

        scale   = scale_fn(elapsed)
        fwd_pwm = MavlinkAPI.percent_to_pwm(fwd_sign * gain * scale)
        lat_pwm = MavlinkAPI.percent_to_pwm(lat_sign * gain * scale)
        api.send_rc_override(forward=fwd_pwm, lateral=lat_pwm)

        if now - last_log >= _LOG_EVERY:
            depth_s = f'{att["depth"]:+.2f}m' if att else 'N/A'
            logger.info(
                f'[{direction[:3].upper():<5}] '
                f't={elapsed:.1f}s  drift={drift:+.1f}°  depth={depth_s}')
            last_log = now

        time.sleep(1.0 / _HZ_LINEAR)

    logger.info(
        f'[{direction[:3].upper():<5}] done  max_drift={max_drift:.1f}°')


# ---------------------------------------------------------------------- #
#  Exit semantics (brake + settle) — owned by each variant                 #
# ---------------------------------------------------------------------- #
def _brake_kick_then_settle(api, direction: str, logger) -> None:
    """Step-variant brake. Constant-gain exits with full velocity, so we
    apply a short reverse kick on the primary axis before settling."""
    axis, pct = _BRAKE[direction]
    pwm       = MavlinkAPI.percent_to_pwm(pct)
    logger.info(
        f'[CMD  ] brake — reverse {axis} {_REV_KICK_PCT}% '
        f'× {_REV_KICK_SEC:.2f}s')
    api.send_rc_override(**{axis: pwm})
    time.sleep(_REV_KICK_SEC)

    api.send_neutral()
    logger.info(f'[CMD  ] settle {_SETTLE_SEC:.1f}s')
    time.sleep(_SETTLE_SEC)


def _brake_settle_only(api, logger) -> None:
    """Ramp-variant brake. The trapezoid ease-out already decelerated us,
    so any reverse kick would over-brake and push the sub backward. Just
    go neutral and let drag + heading-hold do the rest."""
    api.send_neutral()
    logger.info(f'[CMD  ] settle {_SETTLE_SEC:.1f}s (ramp-out = brake)')
    time.sleep(_SETTLE_SEC)
