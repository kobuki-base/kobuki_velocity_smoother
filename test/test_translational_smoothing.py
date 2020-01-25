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
    script_pathname = os.path.join(
        ament_index_python.get_package_prefix('velocity_smoother'),
        'lib',
        'velocity_smoother',
        'test_nodes',
        'translational_command_profile.py'
    )
    return launch_ros.actions.Node(
        package='velocity_smoother',
        node_executable='translational_command_profile.py',
        node_name="commands",
        output='both',
        emulate_tty=True,
        remappings=[
            ('~/cmd_vel', '/raw_cmd_vel'),
        ]
    )

def create_velocity_smoother_node():
    share_dir = ament_index_python.packages.get_package_share_directory('velocity_smoother')

    # There are two different ways to pass parameters to a non-composed node;
    # either by specifying the path to the file containing the parameters, or by
    # passing a dictionary containing the key -> value pairs of the parameters.
    # When starting a *composed* node on the other hand, only the dictionary
    # style is supported.  To keep the code between the non-composed and
    # composed launch file similar, we use that style here as well.
    params_file = os.path.join(share_dir, 'config', 'velocity_smoother_params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['velocity_smoother']['ros__parameters']
    return launch_ros.actions.Node(
        package='velocity_smoother',
        node_executable='velocity_smoother',
        node_name='velocity_smoother',
        output='both',
        parameters=[params],
        remappings=[('smooth_cmd_vel', 'robot_cmd_vel')],
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
            'raw_cmd_vel',
            received_input_data,
            10,
        )
        smoothed_subscriber = node.create_subscription(
            geometry_msgs.msg.Twist,
            'robot_cmd_vel',
            received_smoothed_data,
            10,
        )
        try:
            node.get_logger().info("Waiting for PROFILE_SENT")
            proc_output.assertWaitFor(expected_output="PROFILE_SENT", process=commands, timeout=60)
        finally:
            node.get_logger().info("Raw Timestamps: {}".format(input_timestamps))
            node.get_logger().info("Raw Velocities: {}".format(input_velocities))
            node.get_logger().info("Smoothed Timestamps: {}".format(smoothed_timestamps))
            node.get_logger().info("Smoothed Velocities: {}".format(smoothed_velocities))
            plt.plot(input_timestamps, input_velocities, label="input")
            plt.plot(smoothed_timestamps, smoothed_velocities, label="smooth")
            plt.xlabel('time')
            plt.ylabel('velocity')
            plt.title("Raw Input vs Smoothed Velocities")
            plt.legend()
            plt.show()
            time.sleep(1)
            node.destroy_subscription(input_subscriber)
            node.destroy_subscription(smoothed_subscriber)
