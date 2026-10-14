# This file is part of rclgo
#
# Copyright © 2026 Merlin Drones
#
# Licensed under the Apache License, Version 2.0 (the "License");
#     http://www.apache.org/licenses/LICENSE-2.0

"""Serialize and deserialize test_msgs/Strings and WStrings with rclpy, for
TestStringEdgeCasesMatchRclpy. Reads JSON requests from stdin, one per line:

  {"op": "serialize", "type": "Strings"|"WStrings", "value": <codepoints>}
  {"op": "deserialize", "type": "Strings"|"WStrings", "hex": "<CDR bytes>"}

and prints one JSON reply per line: {"hex": ...} or {"value": <codepoints>}, or
{"error": "<exception type>: <message>"}. Values are lists of code points, so
that NULs and surrogates survive JSON.
"""
import json
import sys

from rclpy.serialization import deserialize_message, serialize_message
from test_msgs.msg import Strings, WStrings

TYPES = {'Strings': (Strings, 'string_value'), 'WStrings': (WStrings, 'wstring_value')}


def handle(req):
    cls, field = TYPES[req['type']]
    if req['op'] == 'serialize':
        msg = cls()
        setattr(msg, field, ''.join(chr(c) for c in req['value']))
        return {'hex': serialize_message(msg).hex()}
    msg = deserialize_message(bytes.fromhex(req['hex']), cls)
    return {'value': [ord(c) for c in getattr(msg, field)]}


def main():
    for line in sys.stdin:
        try:
            reply = handle(json.loads(line))
        except Exception as e:  # noqa: BLE001 - reported to the test
            reply = {'error': f'{type(e).__name__}: {e}'}
        print(json.dumps(reply), flush=True)


if __name__ == '__main__':
    main()
