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

import os
import pytest
import sys
import unittest
import yaml

import ament_index_python
import launch
import launch.actions
import launch_ros
import launch_testing
import launch_testing.asserts

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
#     return launch.actions.ExecuteProcess(
#         cmd=[sys.executable, script_pathname],
#         emulate_tty=True,
#         output='screen'
#     )
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

# @pytest.mark.launch_test
@pytest.mark.rostest    
def generate_test_description(ready_fn):
    command_profile_node = create_command_profile_node()
    velocity_smoother_node = create_velocity_smoother_node()
    return (
        launch.LaunchDescription([
            command_profile_node,
            velocity_smoother_node,
            # Start tests right away - no need to wait for anything
            launch.actions.OpaqueFunction(function=lambda context: ready_fn()),
        ]),
        {
            'profile': command_profile_node,
            'velocity_smoother': velocity_smoother_node
        }
    )

##############################################################################
# Classes
##############################################################################

# These tests will run concurrently with the profiling process.
# After all these tests are done, the launch system will shut
# down the processes that it started up

class TestGoodProcess(unittest.TestCase):

    def test_wait_for_profile_to_be_sent(self):
        # This will match stdout from any process.
        # TODO: lock onto the profiling_process
        self.proc_output.assertWaitFor("PROFILE_SENT", timeout=30)

