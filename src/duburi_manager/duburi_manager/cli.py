#!/usr/bin/env python3
"""
duburi CLI — thin argparse wrapper over DuburiClient.

Usage:
    ros2 run duburi_manager duburi <cmd> [args...]

Examples:
    ros2 run duburi_manager duburi arm
    ros2 run duburi_manager duburi set_mode ALT_HOLD
    ros2 run duburi_manager duburi set_depth -1.5
    ros2 run duburi_manager duburi move_forward 5 80
    ros2 run duburi_manager duburi yaw_left 90
    ros2 run duburi_manager duburi disarm
"""

import argparse
import sys

import rclpy
from rclpy.node import Node

from .client import DuburiClient


def _build_parser() -> argparse.ArgumentParser:
    p   = argparse.ArgumentParser(prog='duburi')
    sub = p.add_subparsers(dest='cmd', required=True)

    # Zero-arg verbs
    for verb in ('arm', 'disarm', 'stop'):
        sub.add_parser(verb)

    # set_mode <MODE>
    m = sub.add_parser('set_mode')
    m.add_argument('mode', help='ArduSub mode (MANUAL, ALT_HOLD, STABILIZE, …)')

    # Linear translations: <duration_s> [gain_%=80]
    for d in ('move_forward', 'move_back', 'move_left', 'move_right'):
        s = sub.add_parser(d)
        s.add_argument('duration', type=float, help='seconds')
        s.add_argument('gain',     type=int, nargs='?', default=80,
                       help='thrust gain %% (default 80)')

    # set_depth <target_m>   (negative = below surface)
    dp = sub.add_parser('set_depth')
    dp.add_argument('target_m', type=float, help='target depth in metres')

    # yaw_{left,right} <degrees>
    for y in ('yaw_left', 'yaw_right'):
        s = sub.add_parser(y)
        s.add_argument('degrees', type=float, help='heading change in degrees')

    return p


def main() -> None:
    args = _build_parser().parse_args()

    rclpy.init()
    node = Node('duburi_cli')
    dc   = DuburiClient(node)

    exit_code = 0
    try:
        dc.wait_for_connection()

        if args.cmd in ('arm', 'disarm', 'stop'):
            getattr(dc, args.cmd)()
        elif args.cmd == 'set_mode':
            dc.set_mode(args.mode)
        elif args.cmd in ('move_forward', 'move_back',
                          'move_left',    'move_right'):
            getattr(dc, args.cmd)(args.duration, args.gain)
        elif args.cmd == 'set_depth':
            dc.set_depth(args.target_m)
        elif args.cmd in ('yaw_left', 'yaw_right'):
            getattr(dc, args.cmd)(args.degrees)

        node.get_logger().info(f'{args.cmd} -> OK')
    except Exception as exc:
        node.get_logger().error(f'{args.cmd} -> FAIL: {exc}')
        exit_code = 1
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
