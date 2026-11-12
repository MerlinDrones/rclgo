# This file is part of rclgo
#
# Copyright © 2026 Merlin Drones
#
# Licensed under the Apache License, Version 2.0 (the "License");
#     http://www.apache.org/licenses/LICENSE-2.0

"""Publish a message given as serialized bytes, every 50 ms until stopped.

Usage: msg_pub.py <package/msg/Type> <topic> <serialized message as hex>
"""
import importlib
import sys

import rclpy
from rclpy.serialization import deserialize_message


def load(type_name):
    pkg, kind, name = type_name.split('/')
    return getattr(importlib.import_module(f'{pkg}.{kind}'), name)


def main():
    msg_type, topic = load(sys.argv[1]), sys.argv[2]
    msg = deserialize_message(bytes.fromhex(sys.argv[3]), msg_type)
    rclpy.init()
    node = rclpy.create_node('rclgo_interop_pub')
    pub = node.create_publisher(msg_type, topic, 10)
    node.create_timer(0.05, lambda: pub.publish(msg))
    print('READY', flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
