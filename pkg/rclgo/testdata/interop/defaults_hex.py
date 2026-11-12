# This file is part of rclgo
#
# Copyright © 2026 Merlin Drones
#
# Licensed under the Apache License, Version 2.0 (the "License");
#     http://www.apache.org/licenses/LICENSE-2.0

"""Print each named test_msgs message default-constructed by rclpy, serialized.

Usage: defaults_hex.py <package/msg/Type>...
Prints "<type> <hex>" per type, for TestDefaultsMatchRclpy.
"""
import importlib
import sys

from rclpy.serialization import serialize_message


def main():
    for name in sys.argv[1:]:
        pkg, kind, cls = name.split('/')
        msg = getattr(importlib.import_module(f'{pkg}.{kind}'), cls)()
        print(name, serialize_message(msg).hex(), flush=True)


if __name__ == '__main__':
    main()
