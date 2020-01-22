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

def setup_module(module):
    print("\n************\nROS Init\n************\n")
    rclpy.init()


def teardown_module(module):
    print("\n*************\nROS Shutdown\n*************\n")
    rclpy.shutdown()

def publish():
    print("Publishing")

##############################################################################
# Tests
##############################################################################


def test_broken():
    node = rclpy.create_node("publisher")
    timer = node.create_timer(
        timer_period_sec=0.1,
        callback=publish
    )

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    count = 0
    try:
        while count < 20:
            executor.spin_once(timeout_sec=0.05)
            count += 1
    except KeyboardInterrupt:
        pass

    print("Shutdown")
    timer.cancel()
    executor.shutdown()
    node.destroy_timer(timer)
    node.destroy_node()
