"""SrotFC -- the SROT board (Hengla firmware) behind the HAL.

Same pymavlink, same MAVLink 2, same sysid 1 / compid 1. Different verbs:

  * **No `RC_CHANNELS_OVERRIDE`.** Actuation is `MANUAL_CONTROL`, x/y/r
    +/-1000 and z 0..1000 with 500 neutral.
  * **`MAV_CMD_SROT_MOVE` (31000)** runs whole motion primitives on the
    board -- ramps, braking, heading hold and depth hold included -- and
    reports progress and one of four terminal results over `COMMAND_ACK`.
  * **Different mode numbers**, and five modes ArduSub has no name for.

Threading follows the same rule as `pixhawk.py`: only the reader thread in
`auv_manager_node` calls `recv_match()`. Everything here reads
`master.messages`, so no getter blocks and none of them can be called from a
context where blocking would matter.

Reference: `JETSON_COMMS.md` (wire contract) and `DUBURI_WS_INTEGRATION.md`
(migration plan) in the srot-control-board repo.
"""

from __future__ import annotations

import math
import os
import time
from typing import Any, Dict, List, Optional, Tuple

os.environ.setdefault('MAVLINK20', '1')   # must precede the pymavlink import
from pymavlink import mavutil             # noqa: E402

from .base import (                                                      # noqa: E402
    MOVE_TERMINAL, FlightController, MoveHandle, MoveResult, Telemetry,
)


SROT_MOVE = 31000

# Wire type codes for SROT_MOVE p1. These are 0-based on the wire; the
# firmware stores `movement::Type = p1 + 1` and NAMED_VALUE_FLOAT MV_TYPE
# reports that internal enum, not p1. Do not compare MV_TYPE against these.
MOVE_TYPES = {
    'forward': 0,
    'back':    1,
    'left':    2,   # strafe left
    'right':   3,   # strafe right
    'turn':    4,
    'dive':    5,
    'stop':    6,
    'hold':    7,
    'style':   8,
    'arc':     9,
}

# The complete FlightMode enum. HEARTBEAT.custom_mode can carry any of these,
# so the reverse map must be exhaustive or get_mode() starts returning
# 'UNKNOWN' for a perfectly valid state.
MODES = {
    'STABILIZE':    0,
    'ACRO':         1,
    'DEPTH_HOLD':   2,
    'SURFACE':      9,
    'MANUAL':       19,
    'MOTOR_DETECT': 20,
    'AUTOTUNE':     21,
    'MOTOR_TUNE':   22,
    'AUTO':         23,
    'STUNT':        100,
    'PATTERN':      101,
}
MODE_NAMES = {v: k for k, v in MODES.items()}

# ArduSub vocabulary the rest of the workspace still speaks. duburi.py has
# YAW_OK_MODES = ('ALT_HOLD', 'POSHOLD', 'GUIDED') and _ensure_alt_hold()
# hard-codes 'ALT_HOLD'; translating here means those paths keep working
# without a rewrite. POSHOLD/GUIDED have no SROT equivalent -- AUTO is the
# closest thing (it is the mode that holds position between primitives).
MODE_ALIASES = {
    'ALT_HOLD': 'DEPTH_HOLD',
    'POSHOLD':  'AUTO',
    'GUIDED':   'AUTO',
    'LOITER':   'AUTO',
}

# Modes the board refuses without a depth sensor fitted, falling back to
# STABILIZE with a STATUSTEXT. set_mode() reports this rather than claiming
# success on a mode that did not take.
DEPTH_REQUIRED_MODES = ('DEPTH_HOLD', 'AUTO', 'PATTERN')

MAV_RESULT_NAMES = {
    0: 'ACCEPTED', 1: 'TEMP_REJECTED', 2: 'DENIED', 3: 'UNSUPPORTED',
    4: 'FAILED', 5: 'IN_PROGRESS', 6: 'CANCELLED',
}

PWM_MIN, PWM_MAX, PWM_MID = 1100, 1900, 1500
NO_OVERRIDE = 65535
STALE_SECONDS = 0.25


def pct_to_mc(percent: float) -> int:
    """-100..100 percent -> MANUAL_CONTROL x/y/r, -1000..1000."""
    return int(max(-100.0, min(100.0, percent)) * 10)


def pct_to_mc_z(percent: float) -> int:
    """-100..100 percent -> MANUAL_CONTROL z, 0..1000 with 500 neutral."""
    return int(500 + max(-100.0, min(100.0, percent)) * 5)


def pwm_to_pct(pwm: int) -> float:
    """1100..1900 -> -100..100. NO_OVERRIDE and None both read as neutral.

    There is no way to express "release this axis" in MANUAL_CONTROL, so a
    released channel becomes 0 %. That is the correct *safe* reading (stop
    driving the axis) but it is NOT the same thing: under RC override the
    autopilot took the axis over, here nobody does. Callers that need a real
    handoff must change mode -- see the note in `base.py`.
    """
    if pwm is None or pwm == NO_OVERRIDE:
        return 0.0
    pwm = max(PWM_MIN, min(PWM_MAX, int(pwm)))
    return (pwm - PWM_MID) / 4.0


class SrotMoveHandle(MoveHandle):
    """Tracks one SROT_MOVE through its COMMAND_ACK stream.

    The board sends IN_PROGRESS immediately, then IN_PROGRESS with progress
    0..99 at ~3 Hz, then exactly one of ACCEPTED / CANCELLED / FAILED /
    DENIED. `update()` polls the message cache; poll it faster than 3 Hz so
    no progress tick is missed.

    TEMP_REJECTED is treated as "not started, try again" rather than a
    terminal result -- the board returns it when it could not take its
    control mutex, and resolving the action on it would abort a move that
    never ran.
    """

    def __init__(self, fc: 'SrotFC', verb: str, seq: int, timeout: float):
        super().__init__(verb)
        self._fc = fc
        self._seq = seq
        self._deadline = time.time() + timeout if timeout > 0 else None
        self.temporarily_rejected = False

    def update(self) -> int:
        if self.result is not None:
            return self.progress

        ack = self._fc._latest_move_ack()
        if ack is not None:
            name = MAV_RESULT_NAMES.get(ack.result, f'RESULT_{ack.result}')
            progress = int(getattr(ack, 'progress', 0) or 0)

            if name == 'IN_PROGRESS':
                self.progress = max(self.progress, min(progress, 99))
            elif name == 'TEMP_REJECTED':
                # Not terminal. Surface it so the caller can retry.
                self.temporarily_rejected = True
            elif name in MOVE_TERMINAL:
                self.progress = 100 if name == 'ACCEPTED' else self.progress
                self.result = MoveResult(
                    result=name,
                    success=(name == 'ACCEPTED'),
                    message=self._explain(name),
                    progress=self.progress,
                )
                return self.progress

        # A vehicle that stops acking (link loss, reboot, safety abort that
        # disarmed mid-stream) must not leave the action pending for ever.
        if self._deadline is not None and time.time() > self._deadline:
            self.result = MoveResult(
                result='FAILED', success=False, progress=self.progress,
                message=(f'{self.verb}: no terminal COMMAND_ACK before the '
                         f'safety timeout -- link lost or the board aborted'))
        return self.progress

    def cancel(self) -> None:
        """Preempt with a `stop`, which brakes and resolves this as CANCELLED."""
        self._fc.move('stop')

    def _explain(self, name: str) -> str:
        if name == 'ACCEPTED':
            return f'{self.verb}: completed'
        if name == 'CANCELLED':
            return f'{self.verb}: preempted by a newer move'
        if name == 'DENIED':
            return (f'{self.verb}: rejected at dispatch -- bad type code or a '
                    f'non-finite parameter')
        txt = self._fc.get_statustext() or ''
        hint = f' ({txt})' if txt else (
            ' -- the board may have refused AUTO for want of a depth sensor')
        return f'{self.verb}: could not start{hint}'


class SrotFC(FlightController):

    name = 'srot'

    # MANUAL_CONTROL carries all four axes every frame. The channel API is
    # emulated and NO_OVERRIDE degrades to neutral; see pwm_to_pct().
    supports_rc_override = False

    # SROT_MOVE runs primitives on the board. This is the whole point.
    supports_native_move = True

    def __init__(self, master, log=None, target_system: int = 1,
                 target_component: int = 1):
        self._m = master
        self._log = log
        self._sys = target_system
        self._comp = target_component
        self._boot = time.time()
        self._move_seq = 0
        self._buttons = 0
        self._warned_release = False
        self._last_autopilot_hb = None

    # ---- logging -------------------------------------------------------
    def _debug(self, msg: str) -> None:
        if self._log is not None:
            self._log.debug(f'[SROT ] {msg}')

    def _warn(self, msg: str) -> None:
        if self._log is not None:
            self._log.warning(f'[SROT ] {msg}')

    # ---- message cache -------------------------------------------------
    def _msg(self, name: str):
        return self._m.messages.get(name)

    def _msg_age(self, name: str) -> Optional[float]:
        msg = self._msg(name)
        if msg is None:
            return None
        ts = getattr(msg, '_timestamp', None)
        if ts is None:
            return 0.0
        return max(0.0, time.time() - ts)

    def _heartbeat(self):
        """Newest HEARTBEAT that is not our own loop-back.

        We send heartbeats as MAV_AUTOPILOT_INVALID; if the link echoes them
        back, taking the newest frame blindly would read our own mode.
        """
        hb = self._msg('HEARTBEAT')
        if hb is not None and hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            self._last_autopilot_hb = hb
            return hb
        return self._last_autopilot_hb

    def _latest_move_ack(self):
        ack = self._msg('COMMAND_ACK')
        if ack is not None and ack.command == SROT_MOVE:
            return ack
        return None

    # ---- arming --------------------------------------------------------
    def arm(self, timeout: float = 15.0) -> Tuple[bool, str]:
        self._m.mav.command_long_send(
            self._sys, self._comp,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)
        ok, reason = self._wait_ack(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout=3.0)
        if not ok:
            return False, reason
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.is_armed():
                return True, 'ACCEPTED'
            time.sleep(0.1)
        return False, 'NOT_ARMED_AFTER_ACK'

    def disarm(self, timeout: float = 15.0) -> Tuple[bool, str]:
        # No ArduSub-style MANUAL/neutral/sleep dance here: SROT stops the
        # thrusters itself on disarm, and its pre-arm/disarm path is not
        # mode-sensitive. Just stop driving, then disarm.
        try:
            self.send_neutral()
        except Exception:
            pass
        self._m.mav.command_long_send(
            self._sys, self._comp,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 0, 0, 0, 0, 0, 0)
        ok, reason = self._wait_ack(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, timeout=3.0)
        if not ok:
            return False, reason
        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self.is_armed():
                return True, 'ACCEPTED'
            time.sleep(0.1)
        return False, 'STILL_ARMED_AFTER_ACK'

    def is_armed(self) -> bool:
        hb = self._heartbeat()
        if hb is None:
            return False
        return bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def _wait_ack(self, command_id: int, timeout: float = 3.0) -> Tuple[bool, str]:
        self._m.messages.pop('COMMAND_ACK', None)
        deadline = time.time() + timeout
        while time.time() < deadline:
            ack = self._msg('COMMAND_ACK')
            if ack is not None and ack.command == command_id:
                name = MAV_RESULT_NAMES.get(ack.result, f'RESULT_{ack.result}')
                return ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED, name
            time.sleep(0.02)
        return False, 'NO_ACK'

    # ---- modes ---------------------------------------------------------
    def set_mode(self, mode_name: str, timeout: float = 8.0) -> Tuple[bool, str]:
        wanted = MODE_ALIASES.get((mode_name or '').upper(), (mode_name or '').upper())
        if wanted not in MODES:
            return False, f'UNKNOWN_MODE:{mode_name}'
        mode_id = MODES[wanted]

        deadline = time.time() + timeout
        while time.time() < deadline:
            self._m.mav.set_mode_send(
                self._sys,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id)
            time.sleep(0.3)
            current = self.get_mode()
            if current == wanted:
                return True, 'ACCEPTED'
            # The board refuses depth-dependent modes with no depth sensor
            # and silently lands in STABILIZE. Detect that rather than
            # spinning until the timeout and reporting a generic failure --
            # the operator needs to know it is the sensor, not the link.
            if wanted in DEPTH_REQUIRED_MODES and current == 'STABILIZE':
                txt = self.get_statustext() or ''
                self._warn(f'{wanted} refused, fell back to STABILIZE. {txt}')
                return False, 'MODE_REFUSED_NO_DEPTH'
        return False, 'MODE_NOT_REACHED'

    def get_mode(self) -> str:
        hb = self._heartbeat()
        if hb is None:
            return 'UNKNOWN'
        return MODE_NAMES.get(hb.custom_mode, f'MODE_{hb.custom_mode}')

    # ---- intent actuation ----------------------------------------------
    def manual(self, fwd: float = 0.0, lat: float = 0.0,
               up: float = 0.0, yaw: float = 0.0) -> None:
        """Normalised axes -1..1 -> MANUAL_CONTROL.

        `up` positive = ascend, matching the board's heave convention.

        Note the board scales this by the live pilot gain (`GAIN`, default
        from `JS_GAIN_DEFAULT` = 0.5). For programmatic control set GAIN to
        1.0 or every command arrives at half authority -- see
        `set_pilot_gain()`.
        """
        self._m.mav.manual_control_send(
            self._sys,
            pct_to_mc(fwd * 100.0),      # x -> forward
            pct_to_mc(lat * 100.0),      # y -> lateral
            pct_to_mc_z(up * 100.0),     # z -> heave, 500 neutral
            pct_to_mc(yaw * 100.0),      # r -> yaw
            self._buttons)

    def set_pilot_gain(self, gain: float) -> None:
        """Set GAIN (0.1..1.0). Use 1.0 for programmatic control."""
        self.set_param('GAIN', max(0.1, min(1.0, float(gain))))

    # ---- native moves --------------------------------------------------
    def move(self, verb: str, primary: float = 0.0, speed: float = 0.0,
             mode: float = 0.0, timeout: float = 0.0, **_: Any) -> MoveHandle:
        """Start a SROT_MOVE primitive. Returns a handle to poll.

        `verb` is a key of MOVE_TYPES. Parameter meanings vary by verb --
        see the p1 table in `JETSON_COMMS.md` §5:

          forward/back/left/right  primary = duration_s, speed = 0..1
          turn                     primary = degrees, speed = deg/s,
                                   mode = 0 relative / 1 absolute
          dive                     primary = depth_m (speed ignored)
          stop / hold / style      primary = -- / seconds / roll count
          arc                      primary = duration_s, speed = 0..1,
                                   mode = signed yaw rate deg/s

        A new move PREEMPTS the running one; there is no queue. Serialise
        goals yourself.
        """
        key = (verb or '').strip().lower()
        if key not in MOVE_TYPES:
            raise ValueError(
                f'unknown move verb {verb!r}. known: {", ".join(sorted(MOVE_TYPES))}')

        # Reject non-finite before it reaches the wire. The board rejects it
        # too (DENIED), but failing here names the offending field.
        for label, value in (('primary', primary), ('speed', speed),
                             ('mode', mode), ('timeout', timeout)):
            if not math.isfinite(float(value)):
                raise ValueError(f'{verb}: {label} is {value!r}, must be finite')

        # Clear any stale ACK so the handle cannot resolve on the previous
        # move's terminal result before this one has even been sent.
        self._m.messages.pop('COMMAND_ACK', None)

        self._m.mav.command_long_send(
            self._sys, self._comp, SROT_MOVE, 0,
            float(MOVE_TYPES[key]), float(primary), float(speed),
            float(mode), float(timeout), 0.0, 0.0)

        self._move_seq += 1
        self._debug(f'move {key} primary={primary} speed={speed} '
                    f'mode={mode} timeout={timeout}')
        # Watchdog: the board's own safety cap plus slack for the ACK to
        # arrive. p5 = 0 means the firmware default of 60 s.
        watchdog = (timeout if timeout > 0 else 60.0) + 10.0
        return SrotMoveHandle(self, key, self._move_seq, watchdog)

    # ---- legacy channel actuation (emulated) ---------------------------
    def _emulated(self, throttle=PWM_MID, yaw=PWM_MID,
                  forward=PWM_MID, lateral=PWM_MID) -> None:
        if not self._warned_release and NO_OVERRIDE in (throttle, yaw, forward, lateral):
            self._warned_release = True
            self._warn('RC channel release (65535) has no MANUAL_CONTROL '
                       'equivalent -- that axis is held at neutral, NOT handed '
                       'to the autopilot. Use set_mode/move for a real handoff.')
        self.manual(
            fwd=pwm_to_pct(forward) / 100.0,
            lat=pwm_to_pct(lateral) / 100.0,
            up=pwm_to_pct(throttle) / 100.0,
            yaw=pwm_to_pct(yaw) / 100.0,
        )

    def send_rc_override(self, pitch: int = PWM_MID, roll: int = PWM_MID,
                         throttle: int = PWM_MID, yaw: int = PWM_MID,
                         forward: int = PWM_MID, lateral: int = PWM_MID) -> None:
        # pitch/roll are accepted and dropped: SROT stabilises both and
        # exposes no pilot axis for them.
        self._emulated(throttle=throttle, yaw=yaw, forward=forward, lateral=lateral)

    def send_rc_translation(self, throttle: int = PWM_MID,
                            forward: int = PWM_MID, lateral: int = PWM_MID) -> None:
        # On ArduSub this deliberately leaves yaw released so HeadingLock
        # owns it. SROT holds heading on-board for the duration of a move,
        # so neutral yaw is the correct emulation.
        self._emulated(throttle=throttle, yaw=PWM_MID,
                       forward=forward, lateral=lateral)

    def send_neutral(self) -> None:
        self.manual(0.0, 0.0, 0.0, 0.0)

    def release_rc_override(self) -> None:
        # Closest honest equivalent: stop driving every axis.
        self.send_neutral()

    def set_target_depth(self, depth_m: float) -> None:
        """Depth setpoint -> a `dive` primitive.

        ArduSub takes a streamed setpoint and runs its own ramp; SROT takes
        the target once and ramps on-board at MOVE_DEPTH_RATE. Streaming
        this at 5 Hz as `motion_depth.py` does still works -- each call
        preempts the last with the same target -- but one call is enough.

        Sign: callers pass ArduSub's convention (negative = below surface);
        SROT's dive primitive wants depth as a positive metre count.
        """
        self.move('dive', primary=abs(float(depth_m)))

    # ---- misc ----------------------------------------------------------
    def set_servo_pwm(self, aux_n: int, pwm: int) -> None:
        """Drive a PCA9685 payload channel. AUX1..AUX6 -> servo 9..14."""
        if not 1 <= aux_n <= 6:
            raise ValueError(f'aux_n must be 1..6, got {aux_n}')
        self._m.mav.command_long_send(
            self._sys, self._comp,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0,
            aux_n + 8, max(PWM_MIN, min(PWM_MAX, int(pwm))), 0, 0, 0, 0, 0)

    def set_relay(self, relay_n: int, on: bool) -> None:
        """Drive an on/off payload channel (PCA9685 relay block)."""
        self._m.mav.command_long_send(
            self._sys, self._comp,
            mavutil.mavlink.MAV_CMD_DO_SET_RELAY, 0,
            int(relay_n), 1 if on else 0, 0, 0, 0, 0, 0)

    def set_param(self, name: str, value: float) -> None:
        self._m.mav.param_set_send(
            self._sys, self._comp, name.encode('ascii')[:16], float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def send_heartbeat(self) -> None:
        """Keep the GCS failsafe fed. Must run at >=1 Hz.

        SROT surfaces the vehicle after GCS_FAILSAFE_MS (5 s) without one.
        """
        self._m.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def set_message_rate(self, message_id: int, hz: float) -> None:
        interval = -1 if hz < 0 else (0 if hz == 0 else int(1e6 / hz))
        self._m.mav.command_long_send(
            self._sys, self._comp,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            message_id, interval, 0, 0, 0, 0, 0)

    # ---- telemetry -----------------------------------------------------
    def get_attitude(self) -> Optional[Dict[str, float]]:
        """Same shape and units as the Pixhawk backend: degrees and metres.

        ATTITUDE carries radians, so it is converted here; depth comes from
        VFR_HUD.alt, which the board already sends as -depth. That matches
        ArduSub's AHRS2.altitude convention (negative below the surface), so
        callers need no sign change.
        """
        att = self._msg('ATTITUDE')
        if att is None:
            return None
        hud = self._msg('VFR_HUD')
        return {
            'yaw':   math.degrees(att.yaw) % 360.0,
            'roll':  math.degrees(att.roll),
            'pitch': math.degrees(att.pitch),
            'depth': float(hud.alt) if hud is not None else 0.0,
        }

    def get_attitude_age(self) -> Optional[float]:
        return self._msg_age('ATTITUDE')

    def get_battery(self) -> Optional[Dict[str, float]]:
        """BATTERY_STATUS id 0 -- the electronics/SBC pack.

        Note `current_battery` is -1 (unknown) on this board unless the
        pack-current shunt has been calibrated in firmware, so 0.0 here
        means "no current sensing", not "no current".
        """
        bat = self._msg('BATTERY_STATUS')
        if bat is None or getattr(bat, 'id', 0) != 0:
            return None
        current = float(bat.current_battery)
        return {
            'voltage': bat.voltages[0] / 1000.0,
            'current': current / 100.0 if current >= 0 else 0.0,
        }

    def get_rc_channels(self) -> Optional[List[int]]:
        # SROT has no RC input path -- there is no radio on the vehicle.
        return None

    def get_statustext(self) -> Optional[str]:
        msg = self._msg('STATUSTEXT')
        return msg.text.strip() if msg is not None else None

    def get_rpm(self) -> List[int]:
        """Per-thruster RPM from bidirectional DShot, or [] if not reporting.

        ESC_STATUS arrives as two messages at 5 Hz, index 0 and index 4.
        pymavlink's cache keeps only the newest of a given type, so this can
        return a half-populated list; treat a 0 as "not reported this tick"
        rather than "stalled".
        """
        esc = self._msg('ESC_STATUS')
        if esc is None:
            return []
        base = int(getattr(esc, 'index', 0) or 0)
        out = [0] * 8
        for i, rpm in enumerate(esc.rpm[:4]):
            if base + i < 8:
                out[base + i] = int(rpm)
        return out

    def telemetry(self) -> Telemetry:
        att = self.get_attitude()
        bat = self.get_battery()

        # BATTERY_STATUS id 1 is the thruster pack, relayed from the 2nd
        # board over ESP-NOW. It is ABSENT, not zero, when that link is not
        # fresh -- so no message means no data, never a flat pack.
        thruster_v = None
        raw = self._msg('BATTERY_STATUS')
        if raw is not None and getattr(raw, 'id', 0) == 1:
            thruster_v = raw.voltages[0] / 1000.0

        return Telemetry(
            armed=self.is_armed(),
            mode=self.get_mode(),
            yaw_deg=att['yaw'] if att else None,
            roll_deg=att['roll'] if att else None,
            pitch_deg=att['pitch'] if att else None,
            depth_m=att['depth'] if att else None,
            battery_voltage=bat['voltage'] if bat else None,
            thruster_voltage=thruster_v,
            rpm=self.get_rpm(),
            attitude_age=self.get_attitude_age(),
        )
