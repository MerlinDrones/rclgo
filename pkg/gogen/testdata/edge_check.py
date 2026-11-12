# This file is part of rclgo
#
# Copyright © 2026 Merlin Drones
#
# Licensed under the Apache License, Version 2.0 (the "License");
#     http://www.apache.org/licenses/LICENSE-2.0

"""Check, from rclpy, a rclgo_test_interfaces/msg/Edge serialized by rclgo.

Usage: edge_check.py <serialized Edge as hex> <serialized default Spellings as hex>
Prints OK if rclpy reads every checked field as rclgo wrote it, and rclgo's
defaults of Spellings are rclpy's.
"""
import sys

from rclgo_test_interfaces.msg import Edge, Spellings
from rclpy.serialization import deserialize_message

m = deserialize_message(bytes.fromhex(sys.argv[1]), Edge)
checks = {
    'type/range/func/x_y_z/xyz': (m.type, m.range, m.func, m.x_y_z, m.xyz) == (1, 2, 3, 4, 5),
    'url2': m.url2 == 'u2',
    'with_default': m.with_default == -7,
    'array_default': list(m.array_default) == [0.5, -0.25, 1e10],
    'strs_default': list(m.strs_default) == ['x', 'y,z', ''],
    'bounded': (m.bounded_str, list(m.bounded_seq), list(m.bounded_strs)) == ('abcde', [-1, 0, 1], ['abcd', 'é']),
    'inner': (m.inner.value, m.inner.name) == (-2**63, 'inner'),
    'inners': [(i.value, i.name) for i in m.inners] == [(1, 'a'), (2, 'b')],
    'inner_seq': [(i.value, i.name) for i in m.inner_seq] == [(3, 'c')],
    'point': (m.point.x, m.point.y, m.point.z) == (1.5, -2.5, 3.25),
    'points': [(p.x, p.y) for p in m.points] == [(1.0, 0.0), (0.0, 2.0)],
    'stamp': (m.stamp.sec, m.stamp.nanosec) == (1700000000, 999999999),
    'wide': m.wide == 'wide é€ 😀',
    'constants': (Edge.C_U64, Edge.C_I64, Edge.C_STR) == (2**64 - 1, -2**63, 'string, with comma'),
}

s, want = deserialize_message(bytes.fromhex(sys.argv[2]), Spellings), Spellings()
for field in want.get_fields_and_field_types():
    value, wanted = getattr(s, field), getattr(want, field)
    checks['Spellings.' + field] = list(value) == list(wanted) if hasattr(wanted, '__len__') else value == wanted

failed = [name for name, ok in checks.items() if not ok]
print('FAILED: ' + ', '.join(failed) if failed else 'OK')
