# This file is part of rclgo
#
# Copyright © 2026 Merlin Drones
#
# Licensed under the Apache License, Version 2.0 (the "License");
#     http://www.apache.org/licenses/LICENSE-2.0

"""Reads, sets and describes parameters of every type on a node with rclpy's
parameter client, for TestParameterValuesOverTheWire.

Usage: param_wire.py <node fully qualified name>
Prints one JSON object per line: {"op": "get"|"set"|"describe", ...}.
"""
import json
import sys

import rclpy
from rclpy.parameter import Parameter, parameter_value_to_python
from rclpy.parameter_client import AsyncParameterClient

NAMES = ['flag', 'count', 'ratio', 'label', 'blob', 'flags', 'counts', 'ratios', 'labels']

NEW_VALUES = [
    Parameter('flag', Parameter.Type.BOOL, False),
    Parameter('count', Parameter.Type.INTEGER, -7),
    Parameter('ratio', Parameter.Type.DOUBLE, 2.5),
    Parameter('label', Parameter.Type.STRING, 'é€ "x"'),
    Parameter('blob', Parameter.Type.BYTE_ARRAY, [b'\x00', b'\xff', b'\x10']),
    Parameter('flags', Parameter.Type.BOOL_ARRAY, [False, True]),
    Parameter('counts', Parameter.Type.INTEGER_ARRAY, [-(2 ** 63), 0, 2 ** 63 - 1]),
    Parameter('ratios', Parameter.Type.DOUBLE_ARRAY, [-1.5, 0.0, 1e300]),
    Parameter('labels', Parameter.Type.STRING_ARRAY, ['', 'a,b', 'ハロー']),
]


def plain(v):
    # rclpy returns a byte array as a list of one-byte bytes objects.
    if isinstance(v, (bytes, bytearray)):
        return v[0] if len(v) == 1 else list(v)
    if isinstance(v, (list, tuple)):
        return [plain(x) for x in v]
    if hasattr(v, 'tolist'):
        return v.tolist()
    return v


def wait(node, future):
    rclpy.spin_until_future_complete(node, future, timeout_sec=10)
    if not future.done():
        print(json.dumps({'op': 'error', 'error': 'timeout'}), flush=True)
        sys.exit(1)
    return future.result()


def main():
    rclpy.init()
    node = rclpy.create_node('rclgo_param_wire')
    client = AsyncParameterClient(node, sys.argv[1])
    if not client.wait_for_services(timeout_sec=10):
        print(json.dumps({'op': 'error', 'error': 'services not ready'}), flush=True)
        sys.exit(1)

    got = wait(node, client.get_parameters(NAMES))
    for name, value in zip(NAMES, got.values):
        print(json.dumps({'op': 'get', 'name': name, 'type': value.type,
                          'value': plain(parameter_value_to_python(value))}), flush=True)

    results = wait(node, client.set_parameters(NEW_VALUES))
    for p, r in zip(NEW_VALUES, results.results):
        print(json.dumps({'op': 'set', 'name': p.name, 'ok': r.successful, 'reason': r.reason}), flush=True)

    got = wait(node, client.get_parameters(NAMES))
    for name, value in zip(NAMES, got.values):
        print(json.dumps({'op': 'get-after-set', 'name': name, 'type': value.type,
                          'value': plain(parameter_value_to_python(value))}), flush=True)

    desc = wait(node, client.describe_parameters(NAMES))
    for d in desc.descriptors:
        print(json.dumps({'op': 'describe', 'name': d.name, 'type': d.type}), flush=True)

    print('DONE', flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
