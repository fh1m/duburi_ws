#!/usr/bin/env python3
"""
Test Runner — Terminal 2.
Executes the Duburi AUV test mission via the /duburi/move ActionServer.

Mission:
  arm → ALT_HOLD → forward 5s → yaw_left 90°
  → depth -2.0m → forward 3s
  → yaw_left 90° → left 3s
  → yaw_left 90° → right 3s
  → yaw_right 90° → back 3s
  → yaw_right 90° → disarm
"""

import rclpy
from rclpy.node import Node
from .client import DuburiClient


def main(args=None):
    rclpy.init(args=args)
    node = Node('duburi_test_runner')
    log  = node.get_logger()
    duburi = DuburiClient(node)

    try:
        duburi.wait_for_connection(timeout=15.0)
        log.info('━' * 52)
        log.info(' DUBURI TEST MISSION — starting')
        log.info('━' * 52)

        duburi.arm()
        duburi.set_mode('MANUAL')
        duburi.set_depth(-0.0, timeout=40.0)

        duburi.move_forward(5.0, gain=60)
        duburi.yaw_left(90.0)

        # duburi.set_depth(-2.0, timeout=40.0)

        # duburi.move_forward(3.0, gain=80)
        # duburi.yaw_left(90.0)
        #
        # duburi.move_left(3.0, gain=80)
        # duburi.yaw_left(90.0)
        #
        # duburi.move_right(3.0, gain=80)
        # duburi.yaw_right(90.0)

        duburi.move_forward(5.0, gain=60)
        duburi.yaw_right(90.0)
        duburi.move_forward(5.0, gain=60)
        duburi.yaw_right(90.0)
        duburi.move_forward(5.0, gain=60)

        duburi.disarm()

        log.info('━' * 52)
        log.info(' DUBURI TEST MISSION — complete ✓')
        log.info('━' * 52)

    except Exception as exc:
        log.error(f'MISSION FAILED: {exc}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
