#!/usr/bin/env python3
"""
AUV Manager Node — Terminal 1.

* Owns MAVLink connection + reader thread (only thread calling recv_match)
* Exposes /duburi/move as a ROS2 ActionServer
* MultiThreadedExecutor: action callbacks + timers run in parallel threads
* Telemetry logger prints only on meaningful state changes
"""

import os, json, threading, time

os.environ['MAVLINK20'] = '1'
from pymavlink import mavutil

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from duburi_interfaces.action import Move
from duburi_control import MavlinkAPI, MovementCommands
from .connection_config import PROFILES, DEFAULT_MODE, NETWORK

_SEP = '━' * 52

# How much a value must change before the status line reprints
_YAW_THRESH   = 5.0    # degrees
_DEPTH_THRESH = 0.08   # metres
_BAT_THRESH   = 0.2    # volts
_FORCE_PRINT  = 30.0   # seconds — always reprint even if nothing changed

# Two small maps keep each command entry trivial to read.
#
# ACK-bearing commands (arm/disarm/set_mode) return (success, reason) where
# reason is a MAV_RESULT name (ACCEPTED, DENIED, NO_ACK, ...). Everything
# else is movement — these just run to completion or raise.
_ACK_DISPATCH = {
    'arm':      lambda mc, r: mc._api.arm(r.timeout or 15),
    'disarm':   lambda mc, r: mc._api.disarm(r.timeout or 20),
    'set_mode': lambda mc, r: mc._api.set_mode(r.target_name, r.timeout or 8),
}

_MOVE_DISPATCH = {
    'stop':         lambda mc, r: mc.stop(),
    'move_forward': lambda mc, r: mc.move_forward(r.duration, int(r.gain or 80)),
    'move_back':    lambda mc, r: mc.move_back(r.duration, int(r.gain or 80)),
    'move_left':    lambda mc, r: mc.move_left(r.duration, int(r.gain or 80)),
    'move_right':   lambda mc, r: mc.move_right(r.duration, int(r.gain or 80)),
    'set_depth':    lambda mc, r: mc.set_depth(r.target, r.timeout or 30),
    'yaw_left':     lambda mc, r: mc.yaw_left(r.target, r.timeout or 30),
    'yaw_right':    lambda mc, r: mc.yaw_right(r.target, r.timeout or 30),
}


class AUVManagerNode(Node):
    def __init__(self):
        super().__init__('duburi_manager')

        self.declare_parameter('mode',          DEFAULT_MODE)
        self.declare_parameter('smooth_yaw',    False)
        self.declare_parameter('smooth_linear', False)
        mode          = self.get_parameter('mode').value
        smooth_yaw    = bool(self.get_parameter('smooth_yaw').value)
        smooth_linear = bool(self.get_parameter('smooth_linear').value)
        profile       = PROFILES.get(mode, PROFILES[DEFAULT_MODE])

        self.get_logger().info(f'Connecting ({mode}) → {profile["conn"]} …')
        kw = {'baud': profile['baud']} if profile['baud'] else {}
        self._master = mavutil.mavlink_connection(profile['conn'], **kw)
        self._master.wait_heartbeat()

        sys_id  = self._master.target_system
        comp_id = self._master.target_component
        yaw_tag = 'ramp' if smooth_yaw    else 'step'
        lin_tag = 'ramp' if smooth_linear else 'step'
        self.get_logger().info(_SEP)
        self.get_logger().info(f' DUBURI AUV MANAGER  │  mode: {mode}')
        self.get_logger().info(
            f' MAVLink: sys={sys_id} comp={comp_id}  (v2.0)')
        self.get_logger().info(
            f' Profiles: yaw={yaw_tag}  linear={lin_tag}')
        if mode in ('pool', 'laptop'):
            self.get_logger().info(
                f' Expect BlueOS "{NETWORK["endpoint"]}" → UDP Client '
                f'{NETWORK["jetson_ip"]}:{NETWORK["mav_port"]}')
        self.get_logger().info(_SEP)

        self._api = MavlinkAPI(self._master)
        self._mc  = MovementCommands(
            self._api,
            logger        = self.get_logger(),
            smooth_yaw    = smooth_yaw,
            smooth_linear = smooth_linear,
        )

        # ---- callback groups -------------------------------------------
        # ReentrantCallbackGroup allows action execute + timers to run in
        # parallel threads under MultiThreadedExecutor
        self._action_cbg = ReentrantCallbackGroup()
        self._timer_cbg  = MutuallyExclusiveCallbackGroup()

        # ---- action server ---------------------------------------------
        self._cmd_active = False
        self._action_server = ActionServer(
            self,
            Move,
            '/duburi/move',
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self._action_cbg,
        )

        # ---- telemetry publisher (for other nodes) ----------------------
        self._pub_state = self.create_publisher(String, '/duburi/state', 10)

        # ---- timers (timer_cbg keeps them mutually exclusive) -----------
        self.create_timer(0.5, self._heartbeat_tick,  callback_group=self._timer_cbg)
        self.create_timer(0.5, self._telemetry_tick,  callback_group=self._timer_cbg)

        # ---- reader thread ---------------------------------------------
        self._last_statustext = ''
        self._prev_state      = {}
        self._last_print_t    = 0.0
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()

    # ------------------------------------------------------------------ #
    #  MAVLink reader — only place recv_match is called                   #
    # ------------------------------------------------------------------ #

    def _reader_loop(self):
        while True:
            # Drain ALL pending messages into master.messages cache
            while self._master.recv_match(blocking=False) is not None:
                pass
            # Forward new STATUSTEXT to logger
            txt = self._api.get_statustext()
            if txt and txt != self._last_statustext:
                self._last_statustext = txt
                self.get_logger().info(f'[ARDUB] {txt}')
            time.sleep(0.005)   # 200 Hz drain

    # ------------------------------------------------------------------ #
    #  Action Server callbacks                                             #
    # ------------------------------------------------------------------ #

    def _goal_cb(self, goal_request):
        if self._cmd_active:
            self.get_logger().warn(
                f'[ACT  ] Rejected {goal_request.cmd} — command already active')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info('[ACT  ] Cancel requested — stopping thrusters')
        self._mc.stop(settle_time=0)
        return CancelResponse.ACCEPT

    def _execute_cb(self, goal_handle):
        req = goal_handle.request
        cmd = req.cmd

        result = Move.Result()

        if cmd not in _ACK_DISPATCH and cmd not in _MOVE_DISPATCH:
            result.success = False
            result.message = f'Unknown command: {cmd}'
            goal_handle.abort()
            return result

        self._cmd_active = True
        self.get_logger().info(f'[ACT  ] {cmd} → EXECUTING')

        done_evt = threading.Event()

        def _feedback_loop():
            while not done_evt.is_set():
                att = self._api.get_attitude()
                if att:
                    fb = Move.Feedback()
                    fb.phase         = 'EXECUTING'
                    fb.current_value = float(att['depth'])
                    fb.error_value   = 0.0
                    fb.status_line   = (
                        f'YAW:{att["yaw"]:.1f}°  '
                        f'DEPTH:{att["depth"]:+.2f}m')
                    goal_handle.publish_feedback(fb)
                done_evt.wait(timeout=0.4)

        fb_thread = threading.Thread(target=_feedback_loop, daemon=True)
        fb_thread.start()

        try:
            if cmd in _ACK_DISPATCH:
                # arm/disarm/set_mode — MAVLink ACK drives success/reason
                ok, reason = _ACK_DISPATCH[cmd](self._mc, req)
                result.success = ok
                result.message = f'{cmd}: {reason}'
            else:
                # Movement commands block until done or raise on failure
                _MOVE_DISPATCH[cmd](self._mc, req)
                result.success = True
                result.message = f'{cmd}: completed'

            att = self._api.get_attitude()
            result.final_value = float(att['depth']) if att else 0.0

            if result.success:
                goal_handle.succeed()
                self.get_logger().info(f'[ACT  ] {cmd} → DONE ({result.message})')
            else:
                goal_handle.abort()
                self.get_logger().error(f'[ACT  ] {cmd} → REJECTED ({result.message})')

        except Exception as exc:
            result.success = False
            result.message = f'{cmd}: exception — {exc}'
            goal_handle.abort()
            self.get_logger().error(f'[ACT  ] {cmd} FAILED: {exc}')
        finally:
            done_evt.set()
            fb_thread.join(timeout=1.0)
            self._cmd_active = False

        return result

    # ------------------------------------------------------------------ #
    #  Timers                                                              #
    # ------------------------------------------------------------------ #

    def _heartbeat_tick(self):
        self._api.send_heartbeat()

    def _telemetry_tick(self):
        att  = self._api.get_attitude()
        bat  = self._api.get_battery()
        rc   = self._api.get_rc_channels()
        mode = self._api.get_mode()
        arm  = self._api.is_armed()

        now = time.time()
        ps  = self._prev_state
        changed = (
            ps.get('arm')  != arm
            or ps.get('mode') != mode
            or abs(ps.get('yaw',   0) - (att['yaw']   if att else 0)) > _YAW_THRESH
            or abs(ps.get('depth', 0) - (att['depth']  if att else 0)) > _DEPTH_THRESH
            or abs(ps.get('bat',   0) - (bat['voltage'] if bat else 0)) > _BAT_THRESH
            or (now - self._last_print_t) > _FORCE_PRINT
        )

        if changed:
            arm_s   = '✓' if arm  else '✗'
            yaw_s   = f'{att["yaw"]:6.1f}°'    if att else '   N/A '
            depth_s = f'{att["depth"]:+6.2f}m'  if att else '   N/A '
            bat_s   = f'{bat["voltage"]:5.1f}V'  if bat else '  N/A '
            self.get_logger().info(
                f'[STATE] ARM:{arm_s} │ {mode:<10} │ '
                f'YAW:{yaw_s} │ DEPTH:{depth_s} │ BAT:{bat_s}')
            self._prev_state = {
                'arm': arm, 'mode': mode,
                'yaw':   att['yaw']    if att else 0,
                'depth': att['depth']  if att else 0,
                'bat':   bat['voltage'] if bat else 0,
            }
            self._last_print_t = now

        # RC line only when any drive channel is non-neutral
        if rc and len(rc) >= 6:
            active = any(abs(rc[i] - 1500) > 50 for i in (2, 3, 4, 5))
            if active:
                self.get_logger().info(
                    f'[RC   ] Thr:{rc[2]}  Yaw:{rc[3]}  Fwd:{rc[4]}  Lat:{rc[5]}')

        # Publish JSON for other nodes
        if att or bat:
            self._pub_state.publish(String(data=json.dumps({
                'armed': arm, 'mode': mode,
                'yaw':     att['yaw']    if att else None,
                'depth':   att['depth']  if att else None,
                'battery': bat['voltage'] if bat else None,
            })))


def main(args=None):
    rclpy.init(args=args)
    node = AUVManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._api.send_neutral()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
