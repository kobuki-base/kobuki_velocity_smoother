#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2020 Daniel Stonier
#
# License: BSD
#   https://raw.githubusercontent.com/kobuki-base/velocity_smoother/license/LICENSE
#
##############################################################################
# Imports
##############################################################################

import rclpy
import rclpy.executors

##############################################################################
# Helpers
##############################################################################

def publish():
    print("Publishing")

##############################################################################
# Tests
##############################################################################


if __name__== "__main__":
    rclpy.init()

    node = rclpy.create_node("publisher")
    timer = node.create_timer(
        timer_period_sec=0.1,
        callback=publish
    )

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        while True:
            executor.spin_once(timeout_sec=0.05)
    except KeyboardInterrupt:
        pass

    print("Shutdown")
    executor.shutdown()
    node.destroy_timer(timer)
    node.destroy_node()
