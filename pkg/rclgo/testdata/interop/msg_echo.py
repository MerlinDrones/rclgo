"""Receive one message of a given type and print it serialized.

Usage: msg_echo.py <package/msg/Type> <topic>
Prints READY once subscribed, then HEX:<serialized message as hex> for the
first message, and exits. rclgo's tests compare the bytes with their own
serialization of the message they sent.
"""
import importlib
import sys

import rclpy
from rclpy.serialization import serialize_message


def load(type_name):
    pkg, kind, name = type_name.split('/')
    return getattr(importlib.import_module(f'{pkg}.{kind}'), name)


def main():
    msg_type, topic = load(sys.argv[1]), sys.argv[2]
    rclpy.init()
    node = rclpy.create_node('rclgo_interop_echo')
    got = []
    node.create_subscription(msg_type, topic, got.append, 10)
    print('READY', flush=True)
    while rclpy.ok() and not got:
        rclpy.spin_once(node, timeout_sec=0.1)
    if got:
        print('HEX:' + serialize_message(got[0]).hex(), flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
