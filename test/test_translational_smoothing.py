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

import functools
import os
import pytest
import sys
import unittest
import typing
import yaml

import ament_index_python
import launch
import launch.actions
import launch_ros
import launch_testing
import launch_testing.asserts

import geometry_msgs.msg as geometry_msgs

##############################################################################
# Helpers
##############################################################################

def qos_profile_latched():
    """
    Convenience retrieval for a latched topic (publisher / subscriber)
    """
    return rclpy.qos.QoSProfile(
        history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        depth=1,
        durability=rclpy.qos.QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
        reliability=rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
    )


def qos_profile_unlatched():
    """
    Default profile for an unlatched topic (in py_trees_ros).
    """
    return rclpy.qos.QoSProfile(
        history=rclpy.qos.QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        depth=1,
        durability=rclpy.qos.QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
        reliability=rclpy.qos.QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE
    )


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
            ('~/odom', '/odometry')
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
        params = yaml.safe_load(f)['velocity_smoother_node']['ros__parameters']
    return launch_ros.actions.Node(
        package='velocity_smoother',
        node_executable='velocity_smoother_node',
        output='both',
        parameters=[params]
    )

@pytest.mark.rostest
def generate_test_description(ready_fn):
    command_profile_node = create_command_profile_node()
    velocity_smoother_node = create_velocity_smoother_node()
    return (
        launch.LaunchDescription([
            command_profile_node,
            velocity_smoother_node,
            # Start tests right away - no need to wait for anything
            launch_testing.actions.ReadyToTest(),
            # launch.actions.OpaqueFunction(function=lambda context: ready_fn()),
        ]),
        {
            'profile': command_profile_node,
            'velocity_smoother': velocity_smoother_node
        }
    )

##############################################################################
# Classes
##############################################################################

def foo_callback(msg):
    print("Got message: {}".format(msg))

# These tests will run concurrently with the profiling process.
# After all these tests are done, the launch system will shut
# down the processes that it started up

class TestCommandProfile(unittest.TestCase):

    def test_wait_for_profile_to_be_sent(self, launch_service, profile, proc_output):
        launch_context = launch_service.context
        node = launch_context.locals.launch_ros_node
        node.get_logger().info("Waiting for Profile")
        proc_output.assertWaitFor(expected_output="PROFILE_SENT", process=profile, timeout=60)

    def test_subscribe_vel_topics(self, launch_service, command_profile_node):
        launch_context = launch_service.context
        node = launch_context.locals.launch_ros_node
        raw_velocities = []
        raw_timestamps = []
        smoothed_velocities = []
        smoothed_timestamps = []

        def append_velocity(
                velocity_container,  # typing.List[float]
                timestamps_container,  # typing.List[float]
                msg  # geometry_msgs.Twist
        ):
            velocity_container.append(msg.linear.x)
            timestamps_container.append(rclpy.clock.Clock.now())

        raw_subscriber = node.create_subscription(
            geometry_msgs.Twist,
            '/raw_cmd_vel',
            functools.partial(append_velocity, raw_velocities, raw_timestamps),
            10
        )
        smoothed_subscriber = node.create_subscription(
            geometry_msgs.Twist,
            '/smooth_cmd_vel',
            functools.partial(append_velocity, smoothed_velocities, smoothed_timestamps),
            10
        )
        foo_subscriber = node.create_subscription(
            geometry_msgs.Twist,
            '/foo',
            foo_callback,
#            qos_profile=qos_profile_unlatched()
        )
        try:
            node.get_logger().info("Waiting for Profile")
            proc_output.assertWaitFor(expected_output="PROFILE_SENT", process=profile, timeout=60)
        finally:
            node.get_logger().info("Raw Timestamps: {}".format(raw_timestamps))
            node.destroy_subscription(raw_subscriber)
            node.destroy_subscription(smoothed_subscriber)
