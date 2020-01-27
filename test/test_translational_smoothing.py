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

import matplotlib
# choose a backend that lets it construct plots off the main thread
#  https://stackoverflow.com/questions/49921721/runtimeerror-main-thread-is-not-in-main-loop-with-matplotlib-and-flask
matplotlib.use('Agg')

import functools
import matplotlib.pyplot as plt
import os
import sys
import time
import unittest
import uuid
import yaml

import ament_index_python
import launch
import launch_ros
import launch_ros.actions
import launch_testing.actions
import launch_testing_ros
import rclpy
import rclpy.qos

import pytest

import geometry_msgs.msg
import std_msgs.msg

##############################################################################
# Helpers
##############################################################################


def create_command_profile_node():
    return launch_ros.actions.Node(
        package='velocity_smoother',
        node_executable='translational_command_profile.py',
        node_name="commands",
        output='both',
        emulate_tty=True,
        remappings=[
            ('~/actual/cmd_vel', '/cmd_vel'),
            ('~/generated/cmd_vel', '~/cmd_vel'),
            ('~/odometry', '/odometry'),
        ],
    )

def create_velocity_smoother_node():
    # Stage the test so that it's slow on the ramp up, fine on the ramp down
    # Switch between the three variants of feedback, results should remain the same
    parameters = {}
    parameters['speed_lim_v'] = 0.8
    parameters['speed_lim_w'] = 5.4
    parameters['accel_lim_v'] = 0.125
    parameters['accel_lim_w'] = 3.5
    parameters['frequency'] = 20.0
    parameters['decel_factor'] = 2.0
    parameters['feedback'] = 1  # 1 - ODOMETRY, 2 - COMMANDS (velocity_smoother.hpp)
    return launch_ros.actions.Node(
        package='velocity_smoother',
        node_executable='velocity_smoother',
        node_name='velocity_smoother',
        output='both',
        parameters=[parameters],
        remappings=[
            ('~/feedback/odometry', '/odometry'),
            ('~/feedback/cmd_vel', '/cmd_vel'),
            ('~/input', '/commands/cmd_vel'),
            ('~/smoothed', '/cmd_vel'),
        ],
    )

@pytest.mark.rostest
def generate_test_description():
    # Normally, talker publishes on the 'chatter' topic and listener listens on the
    # 'chatter' topic, but we want to show how to use remappings to munge the data so we
    # will remap these topics when we launch the nodes and insert our own node that can
    # change the data as it passes through
    path_to_test = os.path.dirname(__file__)

    command_profile_node = create_command_profile_node()
    velocity_smoother_node = create_velocity_smoother_node()

    return (
        launch.LaunchDescription([
            command_profile_node,
            velocity_smoother_node,
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
        ]),
        {
            'commands': command_profile_node,
            'velocity_smoother': velocity_smoother_node,
        }
    )

##############################################################################
# Classes
##############################################################################

# These tests will run concurrently with the profiling process.
# After all these tests are done, the launch system will shut
# down the processes that it started up

class TestCommandProfile(unittest.TestCase):

    def test_subscribe_vel_topics(self, launch_service, commands, proc_output):
        launch_context = launch_service.context
        node = launch_context.locals.launch_ros_node
        input_velocities = []
        input_timestamps = []
        smoothed_velocities = []
        smoothed_timestamps = []

        def received_input_data(msg):
            # node.get_logger().warn("received input data {}".format(msg))
            input_velocities.append(msg.linear.x)
            input_timestamps.append(time.monotonic())

        def received_smoothed_data(msg):
            # node.get_logger().warn("received smoothed data {}".format(msg))
            smoothed_velocities.append(msg.linear.x)
            smoothed_timestamps.append(time.monotonic())

        input_subscriber = node.create_subscription(
            geometry_msgs.msg.Twist,
            '/commands/cmd_vel',
            received_input_data,
            10,
        )
        smoothed_subscriber = node.create_subscription(
            geometry_msgs.msg.Twist,
            '/cmd_vel',
            received_smoothed_data,
            10,
        )
        try:
            node.get_logger().info("Waiting for PROFILE_SENT")
            proc_output.assertWaitFor(expected_output="PROFILE_SENT", process=commands, timeout=60)
        finally:
            self.assertAlmostEqual(0.5, max(input_velocities))
            self.assertTrue(0.5 > max(smoothed_velocities))
            self.assertTrue(0.4 < max(smoothed_velocities))
            # plot a graph for easy viz
            plt.plot(input_timestamps, input_velocities, label="input")
            plt.plot(smoothed_timestamps, smoothed_velocities, label="smooth")
            plt.xlabel('time')
            plt.ylabel('velocity')
            plt.title("Raw Input vs Smoothed Velocities")
            plt.legend()
            plt.savefig('profiles.png')
            node.destroy_subscription(input_subscriber)
            node.destroy_subscription(smoothed_subscriber)
