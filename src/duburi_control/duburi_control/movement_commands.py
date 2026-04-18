#!/usr/bin/env python3
"""
MovementCommands — thin facade over the per-axis modules.

All commands serialize via threading.Lock (stop → execute → stop).
The class itself does only three things:
  1. lock / state / logging plumbing
  2. neutral-thrust stop between commands
  3. dispatch to step-or-ramp variant per axis based on the two flags
     `smooth_yaw` and `smooth_linear` (both default False = bang-bang).

Axis-level behaviour lives in:
  movement_yaw.py     → yaw_step    / yaw_ramp
  movement_linear.py  → linear_step / linear_ramp   (each owns its own brake)
  movement_depth.py   → depth_hold

Pure math (smootherstep, trapezoid_ramp) lives in motion_profiles.py.

Defaults mean a plain `ros2 run duburi_manager auv_manager` replays the
previously-validated "very good mission" behaviour byte-for-byte; set
  -p smooth_yaw:=true     → smoothed yaw only
  -p smooth_linear:=true  → smoothed linear only
to A/B each axis in isolation.
"""

import logging
import threading
import time

from .mavlink_api      import MavlinkAPI
from .movement_pids    import DepthPID
from .movement_yaw     import yaw_step,    yaw_ramp
from .movement_linear  import linear_step, linear_ramp
from .movement_depth   import depth_hold


class MovementCommands:
    """Serialised movement facade.

    Cross-command isolation contract
    --------------------------------
    Every public command (move_*, yaw_*, set_depth) runs under self._lock
    so only one is active at a time. Between commands, state is reset via
    `stop()` (or the variant's own brake+settle for linear moves):

      * RC override channels 1..6  → forced to neutral 1500 (active override).
        This supersedes any lingering SET_ATTITUDE_TARGET from a prior yaw.
      * DepthPID integral / last-error → reset to 0.
      * COMMAND_ACK cache  → cleared per ACK-bearing command in mavlink_api.
      * Flight mode        → persisted (set_depth auto-engages ALT_HOLD).
      * Arm state          → persisted (explicit arm/disarm only).

    Exit semantics are owned by the axis module:
      * linear_step → aggressive reverse kick + settle (full-velocity exit)
      * linear_ramp → settle only (ramp-out IS the brake)
      * yaw_*       → neutral stop for 0.3 s (ArduSub heading-hold latches)
      * depth_hold  → neutral stop for 0.3 s (ALT_HOLD keeps altitude)

    Parameters
    ----------
    api : MavlinkAPI
    logger : optional ROS logger (falls back to stdlib logging.getLogger).
    smooth_yaw : bool
        False → yaw_step (bang-bang, ArduSub onboard PID drives profile).
        True  → yaw_ramp (smootherstep setpoint sweep, no overshoot).
    smooth_linear : bool
        False → linear_step (constant gain + reverse-kick brake).
        True  → linear_ramp (trapezoid_ramp thrust envelope, settle-only brake).
    """

    def __init__(self, api: MavlinkAPI, logger=None, *,
                 smooth_yaw: bool = False,
                 smooth_linear: bool = False):
        self._api             = api
        self._log             = logger or logging.getLogger(__name__)
        self._lock            = threading.Lock()
        self._smooth_yaw      = smooth_yaw
        self._smooth_linear   = smooth_linear
        self._depth_pid       = DepthPID()

    # ================================================================= #
    #  Stops — shared by every command                                    #
    # ================================================================= #

    def stop(self, settle_time: float = 0.6):
        """Neutral-thrust stop. All channels 1500 engages ArduSub's
        heading hold and alt-hold latch. Resets DepthPID integrator so
        the next depth command starts from a clean slate."""
        self._api.send_neutral()
        self._depth_pid.reset()
        self._log_msg('[CMD  ] STOP — stabilising...')
        time.sleep(settle_time)

    # ================================================================= #
    #  Linear                                                             #
    # ================================================================= #

    def move_forward(self, duration: float, gain: int = 80):
        self._run_linear('forward', duration, gain)

    def move_back(self, duration: float, gain: int = 80):
        self._run_linear('back', duration, gain)

    def move_left(self, duration: float, gain: int = 80):
        self._run_linear('left', duration, gain)

    def move_right(self, duration: float, gain: int = 80):
        self._run_linear('right', duration, gain)

    def _run_linear(self, direction: str, duration: float, gain: int):
        with self._lock:
            self.stop()
            fn   = linear_ramp if self._smooth_linear else linear_step
            mode = 'RAMP'      if self._smooth_linear else 'STEP'
            self._log_msg(
                f'[CMD  ] move_{direction}  {duration:.1f}s  '
                f'gain={gain}%  ({mode})')
            fn(self._api, direction, duration, gain, self._log)
            self._depth_pid.reset()

    # ================================================================= #
    #  Yaw                                                                #
    # ================================================================= #

    def yaw_left(self, degrees: float, timeout: float = 30.0):
        self._run_yaw(-abs(degrees), timeout, 'LEFT')

    def yaw_right(self, degrees: float, timeout: float = 30.0):
        self._run_yaw(+abs(degrees), timeout, 'RIGHT')

    def _run_yaw(self, signed_degrees: float, timeout: float, label: str):
        with self._lock:
            self.stop()
            att        = self._api.get_attitude()
            start_yaw  = att['yaw'] if att else 0.0
            target_yaw = (start_yaw + signed_degrees) % 360
            fn = yaw_ramp if self._smooth_yaw else yaw_step
            fn(self._api, start_yaw, target_yaw, timeout, label, self._log)
            self.stop(settle_time=0.3)

    # ================================================================= #
    #  Depth                                                              #
    # ================================================================= #

    def set_depth(self, target_m: float, timeout: float = 30.0):
        with self._lock:
            self.stop()
            self._log_msg(f'[CMD  ] set_depth  {target_m:.2f}m')
            ok, reason = self._api.set_mode('ALT_HOLD')
            if not ok:
                self._log_msg(f'[DEPTH] ⚠ set_mode ALT_HOLD: {reason}')
            depth_hold(self._api, self._depth_pid,
                       target_m, timeout, self._log)
            self.stop(settle_time=0.3)

    # ─────────────────────────────────────────────────────────────────
    def _log_msg(self, msg: str):
        if hasattr(self._log, 'info'):
            self._log.info(msg)
        else:
            print(msg)
