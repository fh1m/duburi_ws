#!/usr/bin/env python3
"""
Raw MAVLink layer — only file that touches pymavlink.

THREADING RULE: Only the reader thread (in auv_manager_node) calls recv_match().
All methods here read from master.messages cache only. This prevents message
consumption races between the reader and worker threads.
"""

import os
import math
import time

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil
from pymavlink.quaternion import QuaternionBase

_CH_PITCH    = 0
_CH_ROLL     = 1
_CH_THROTTLE = 2
_CH_YAW      = 3
_CH_FORWARD  = 4
_CH_LATERAL  = 5
_NO_OVERRIDE = 65535

# Human-readable names for COMMAND_ACK.result values. Anything that isn't
# ACCEPTED is surfaced straight to the action server's result.message so the
# operator sees *why* a command failed, not just that it did.
_MAV_RESULT = {
    0: 'ACCEPTED',
    1: 'TEMP_REJECTED',
    2: 'DENIED',
    3: 'UNSUPPORTED',
    4: 'FAILED',
    5: 'IN_PROGRESS',
    6: 'CANCELLED',
}


class MavlinkAPI:
    def __init__(self, master):
        self._master = master
        self._boot_time = time.time()

    # ------------------------------------------------------------------ #
    #  COMMAND_ACK — fast, explicit failure reporting                     #
    # ------------------------------------------------------------------ #

    def _clear_ack(self):
        """Drop any cached COMMAND_ACK so wait_ack only sees fresh replies."""
        self._master.messages.pop('COMMAND_ACK', None)

    def wait_ack(self, command_id: int, timeout: float = 3.0) -> tuple[bool, str]:
        """
        Poll master.messages for a COMMAND_ACK matching ``command_id``.

        Returns (accepted, reason_name). ``accepted`` is True only for
        MAV_RESULT_ACCEPTED (0). On timeout returns (False, 'NO_ACK').

        Must be paired with ``_clear_ack()`` before sending the command,
        otherwise a stale ACK from a previous command can be returned.
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            ack = self._master.messages.get('COMMAND_ACK')
            if ack is not None and ack.command == command_id:
                name = _MAV_RESULT.get(ack.result, f'RESULT_{ack.result}')
                return ack.result == 0, name
            time.sleep(0.05)
        return False, 'NO_ACK'

    # ------------------------------------------------------------------ #
    #  Arm / Disarm — ACK for rejection, heartbeat poll for completion    #
    # ------------------------------------------------------------------ #

    def arm(self, timeout: float = 15.0) -> tuple[bool, str]:
        """
        Returns (success, reason). Reason is a MAV_RESULT name (ACCEPTED,
        DENIED, FAILED, ...) or 'NO_ACK' / 'NOT_ARMED_AFTER_ACK'.

        ArduSub ACKs the command before the arm actually completes (pre-arm
        checks run in parallel), so we still poll is_armed() after.
        """
        cmd = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        self._clear_ack()
        self._master.mav.command_long_send(
            self._master.target_system, self._master.target_component,
            cmd, 0, 1, 0, 0, 0, 0, 0, 0)

        accepted, reason = self.wait_ack(cmd, timeout=3.0)
        if not accepted:
            return False, reason

        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.is_armed():
                return True, 'ACCEPTED'
            time.sleep(0.1)
        return False, 'NOT_ARMED_AFTER_ACK'

    def disarm(self, timeout: float = 15.0) -> tuple[bool, str]:
        """
        Swaps to MANUAL and neutralises thrusters before disarming — this is
        what QGC does and avoids ArduSub's "still moving" disarm rejection.
        """
        self.set_mode('MANUAL')
        time.sleep(3)
        self.send_neutral()

        cmd = mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        self._clear_ack()
        self._master.mav.command_long_send(
            self._master.target_system, self._master.target_component,
            cmd, 0, 0, 0, 0, 0, 0, 0, 0)

        accepted, reason = self.wait_ack(cmd, timeout=3.0)
        if not accepted:
            return False, reason

        deadline = time.time() + timeout
        while time.time() < deadline:
            if not self.is_armed():
                return True, 'ACCEPTED'
            time.sleep(0.1)
        return False, 'STILL_ARMED_AFTER_ACK'

    # ------------------------------------------------------------------ #
    #  Mode — SET_MODE is a legacy message with no COMMAND_ACK, so we     #
    #  retry the send and poll the heartbeat cache for the actual change. #
    # ------------------------------------------------------------------ #

    def set_mode(self, mode_name: str, timeout: float = 8.0) -> tuple[bool, str]:
        """Returns (success, reason). Reason is 'ACCEPTED' or 'MODE_NOT_REACHED'."""
        mode_id = self._master.mode_mapping().get(mode_name)
        if mode_id is None:
            return False, f'UNKNOWN_MODE:{mode_name}'
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._master.mav.set_mode_send(
                self._master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id)
            time.sleep(0.3)
            if self.get_mode() == mode_name:
                return True, 'ACCEPTED'
        return False, 'MODE_NOT_REACHED'

    # ------------------------------------------------------------------ #
    #  RC Override                                                         #
    # ------------------------------------------------------------------ #

    def send_rc_override(self, pitch=1500, roll=1500, throttle=1500,
                         yaw=1500, forward=1500, lateral=1500):
        vals = [_NO_OVERRIDE] * 18
        vals[_CH_PITCH]    = int(pitch)
        vals[_CH_ROLL]     = int(roll)
        vals[_CH_THROTTLE] = int(throttle)
        vals[_CH_YAW]      = int(yaw)
        vals[_CH_FORWARD]  = int(forward)
        vals[_CH_LATERAL]  = int(lateral)
        self._master.mav.rc_channels_override_send(
            self._master.target_system, self._master.target_component, *vals)

    def send_rc_translation(self, throttle=1500, forward=1500, lateral=1500):
        """
        Override translation channels only (Ch3 throttle, Ch5 forward, Ch6 lateral).
        Leaves pitch/roll/yaw RC channels released (65535) so SET_ATTITUDE_TARGET
        or ArduSub's internal stabilisers retain authority over attitude.

        Use during yaw commands that are driven by set_attitude_setpoint() —
        if Ch4 were overridden here, even to 1500, ArduSub would prefer the
        pilot's rate command over the attitude target.
        """
        vals = [_NO_OVERRIDE] * 18
        vals[_CH_THROTTLE] = int(throttle)
        vals[_CH_FORWARD]  = int(forward)
        vals[_CH_LATERAL]  = int(lateral)
        self._master.mav.rc_channels_override_send(
            self._master.target_system, self._master.target_component, *vals)

    def send_neutral(self):
        self.send_rc_override(1500, 1500, 1500, 1500, 1500, 1500)

    def release_rc_override(self):
        """Send all channels as 65535 to fully release pilot override."""
        vals = [_NO_OVERRIDE] * 18
        self._master.mav.rc_channels_override_send(
            self._master.target_system, self._master.target_component, *vals)

    # ------------------------------------------------------------------ #
    #  Heartbeat (mandatory ≥ 1 Hz)                                       #
    # ------------------------------------------------------------------ #

    def send_heartbeat(self):
        self._master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0)

    # ------------------------------------------------------------------ #
    #  Setpoints                                                           #
    # ------------------------------------------------------------------ #

    def set_attitude_setpoint(self, roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
        """
        Command ArduSub's internal 400 Hz attitude stabiliser.
        Works in ALT_HOLD / STABILIZE modes when Ch4 RC override is released.
        Stream at ≥ 5 Hz to keep the target active (ArduSub drops it after ~1 s
        of silence and falls back to pilot RC).

        Mask ignores body-rate fields (we're commanding an absolute attitude,
        not rates) and throttle (so ALT_HOLD can keep holding depth).
        """
        mask = (
            mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
            | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
            | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
            | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE
        )
        q = QuaternionBase([math.radians(a) for a in (roll_deg, pitch_deg, yaw_deg)])
        self._master.mav.set_attitude_target_send(
            int(1e3 * (time.time() - self._boot_time)),
            self._master.target_system, self._master.target_component,
            mask, q, 0, 0, 0, 0)

    def set_servo_pwm(self, servo_n: int, us: int):
        self._master.mav.command_long_send(
            self._master.target_system, self._master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0, servo_n, us, 0, 0, 0, 0, 0)

    # ------------------------------------------------------------------ #
    #  Telemetry reads — master.messages cache only (non-blocking)        #
    # ------------------------------------------------------------------ #

    def get_attitude(self):
        msg = self._master.messages.get('AHRS2')
        if msg is None:
            return None
        yaw_deg = math.degrees(msg.yaw)
        if yaw_deg < 0:
            yaw_deg += 360.0
        return {
            'yaw':   yaw_deg,
            'roll':  math.degrees(msg.roll),
            'pitch': math.degrees(msg.pitch),
            'depth': msg.altitude,
        }

    def get_battery(self):
        msg = self._master.messages.get('BATTERY_STATUS')
        if msg is None:
            return None
        return {
            'voltage': msg.voltages[0] / 1000.0,
            'current': msg.current_battery / 100.0,
        }

    def get_rc_channels(self):
        msg = self._master.messages.get('RC_CHANNELS')
        if msg is None:
            return None
        return [
            msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
            msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw,
        ]

    def get_statustext(self) -> str | None:
        msg = self._master.messages.get('STATUSTEXT')
        return msg.text.strip() if msg else None

    def get_mode(self) -> str:
        msg = self._master.messages.get('HEARTBEAT')
        if msg is None:
            return 'UNKNOWN'
        mode_map = {v: k for k, v in self._master.mode_mapping().items()}
        return mode_map.get(msg.custom_mode, str(msg.custom_mode))

    def is_armed(self) -> bool:
        msg = self._master.messages.get('HEARTBEAT')
        if msg is None:
            return False
        return bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    # ------------------------------------------------------------------ #
    #  Utilities                                                           #
    # ------------------------------------------------------------------ #

    @staticmethod
    def percent_to_pwm(pct: float) -> int:
        """Convert -100..100 percent to 1100..1900 µs PWM. 0 → 1500."""
        return max(1100, min(1900, int(1500 + (pct / 100.0) * 400)))

    @staticmethod
    def heading_error(target: float, current: float) -> float:
        """Shortest-path error on 0-360° circle. Returns -180..180."""
        return (target - current + 540) % 360 - 180
