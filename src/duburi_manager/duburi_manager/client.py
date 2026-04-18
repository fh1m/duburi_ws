#!/usr/bin/env python3
"""
DuburiClient — blocking Python API over the /duburi/move ActionServer.
All public methods block until the goal succeeds, fails, or times out.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from duburi_interfaces.action import Move


class DuburiClient:
    def __init__(self, node: Node):
        self._node   = node
        self._client = ActionClient(node, Move, '/duburi/move')

    # ------------------------------------------------------------------ #
    #  Connection                                                          #
    # ------------------------------------------------------------------ #

    def wait_for_connection(self, timeout: float = 15.0):
        self._node.get_logger().info('Waiting for /duburi/move action server...')
        if not self._client.wait_for_server(timeout_sec=timeout):
            raise TimeoutError('/duburi/move action server not available')
        self._node.get_logger().info('Action server ready.')

    # ------------------------------------------------------------------ #
    #  Commands — each maps to one Move.action goal                       #
    # ------------------------------------------------------------------ #

    def arm(self, timeout: float = 15.0):
        self._goal('arm', timeout=timeout)

    def disarm(self, timeout: float = 20.0):
        self._goal('disarm', timeout=timeout)

    def set_mode(self, mode: str, timeout: float = 8.0):
        self._goal('set_mode', target_name=mode, timeout=timeout)

    def stop(self):
        self._goal('stop')

    def move_forward(self, duration: float, gain: int = 80):
        self._goal('move_forward', duration=float(duration), gain=float(gain))

    def move_back(self, duration: float, gain: int = 80):
        self._goal('move_back', duration=float(duration), gain=float(gain))

    def move_left(self, duration: float, gain: int = 80):
        self._goal('move_left', duration=float(duration), gain=float(gain))

    def move_right(self, duration: float, gain: int = 80):
        self._goal('move_right', duration=float(duration), gain=float(gain))

    def set_depth(self, target_m: float, timeout: float = 30.0):
        self._goal('set_depth', target=float(target_m), timeout=float(timeout))

    def yaw_left(self, degrees: float, timeout: float = 30.0):
        self._goal('yaw_left', target=float(degrees), timeout=float(timeout))

    def yaw_right(self, degrees: float, timeout: float = 30.0):
        self._goal('yaw_right', target=float(degrees), timeout=float(timeout))

    # ------------------------------------------------------------------ #
    #  Internal                                                            #
    # ------------------------------------------------------------------ #

    def _goal(self, cmd: str, **kwargs):
        """Send a Move goal and block until it completes or raises."""
        goal = Move.Goal()
        goal.cmd = cmd
        for k, v in kwargs.items():
            setattr(goal, k, v)

        # Send goal and wait for it to be accepted
        future = self._client.send_goal_async(goal, feedback_callback=self._on_feedback)
        rclpy.spin_until_future_complete(self._node, future)
        handle = future.result()

        if not handle.accepted:
            raise RuntimeError(f'Goal "{cmd}" was REJECTED by action server')

        # Wait for result
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self._node, result_future)
        result = result_future.result().result

        if not result.success:
            raise RuntimeError(f'Goal "{cmd}" FAILED: {result.message}')

    def _on_feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        self._node.get_logger().debug(
            f'[FB] {fb.phase}  {fb.status_line}')
