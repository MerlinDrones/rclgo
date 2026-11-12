# This file is part of rclgo
#
# Copyright © 2026 Merlin Drones
#
# Licensed under the Apache License, Version 2.0 (the "License");
#     http://www.apache.org/licenses/LICENSE-2.0

"""One side of the QoS compatibility matrix, in rclpy.

Usage: qos_matrix.py <namespace>
For every reliability and durability, publishes "matrix" at 20 Hz on
<namespace>/py_<reliability>_<durability>, and subscribes to
<namespace>/go_<reliability>_<durability> with every reliability and
durability, printing "RECV <topic> <requested>" the first time each
subscription receives a message.
"""
import sys

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

RELIABILITIES = {'reliable': ReliabilityPolicy.RELIABLE, 'best_effort': ReliabilityPolicy.BEST_EFFORT}
DURABILITIES = {'volatile': DurabilityPolicy.VOLATILE, 'transient_local': DurabilityPolicy.TRANSIENT_LOCAL}


def profiles():
    for rel_name, rel in RELIABILITIES.items():
        for dur_name, dur in DURABILITIES.items():
            yield f'{rel_name}_{dur_name}', QoSProfile(depth=10, reliability=rel, durability=dur)


def main():
    namespace = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node('rclgo_interop_qos_matrix')
    publishers = [node.create_publisher(String, f'{namespace}/py_{name}', qos) for name, qos in profiles()]
    msg = String(data='matrix')
    node.create_timer(0.05, lambda: [p.publish(msg) for p in publishers])
    seen = set()

    def on_message(topic, requested):
        def callback(_):
            if (topic, requested) not in seen:
                seen.add((topic, requested))
                print(f'RECV {topic} {requested}', flush=True)
        return callback

    for offered, _ in profiles():
        topic = f'go_{offered}'
        for requested, qos in profiles():
            node.create_subscription(String, f'{namespace}/{topic}', on_message(topic, requested), qos)
    print('READY', flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
