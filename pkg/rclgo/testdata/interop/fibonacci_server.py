# This file is part of rclgo
#
# Copyright © 2026 Merlin Drones
#
# Licensed under the Apache License, Version 2.0 (the "License");
#     http://www.apache.org/licenses/LICENSE-2.0

"""An rclpy action server for example_interfaces/action/Fibonacci.

Usage: fibonacci_server.py <action name>
Publishes feedback after each element and supports cancellation, like the
action_tutorials server.
"""
import sys
import time

import rclpy
from example_interfaces.action import Fibonacci
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import MultiThreadedExecutor


def execute(goal_handle):
    seq = [0, 1]
    feedback = Fibonacci.Feedback()
    for i in range(1, goal_handle.request.order):
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = Fibonacci.Result()
            result.sequence = seq
            return result
        seq.append(seq[i] + seq[i - 1])
        feedback.sequence = seq
        goal_handle.publish_feedback(feedback)
        time.sleep(0.05)
    goal_handle.succeed()
    result = Fibonacci.Result()
    result.sequence = seq
    return result


def main():
    rclpy.init()
    node = rclpy.create_node('rclgo_interop_fibonacci_server')
    ActionServer(node, Fibonacci, sys.argv[1], execute_callback=execute,
                 cancel_callback=lambda _: CancelResponse.ACCEPT)
    print('READY', flush=True)
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
