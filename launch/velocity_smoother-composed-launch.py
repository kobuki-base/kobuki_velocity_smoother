#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2020 Open Source Robotics Foundation, Inc.
# 
# Software License Agreement (BSD License 2.0)
#   https://raw.githubusercontent.com/kobuki-base/velocity_smoother/license/LICENSE

"""Launch the velocity smoother as a composed node with default configuration."""

import os

import ament_index_python.packages

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

import yaml


def generate_launch_description():
    share_dir = ament_index_python.packages.get_package_share_directory('velocity_smoother')

    # Passing parameters to a composed node must be done via a dictionary of
    # key -> value pairs.  Here we read in the data from the configuration file
    # and create a dictionary of it that the ComposableNode will accept.
    params_file = os.path.join(share_dir, 'config', 'velocity_smoother_params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['velocity_smoother']['ros__parameters']
    container = ComposableNodeContainer(
            node_name='velocity_smoother_container',
            node_namespace='',
            package='rclcpp_components',
            node_executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='velocity_smoother',
                    node_plugin='velocity_smoother::VelocitySmoother',
                    node_name='velocity_smoother',
                    parameters=[params]),
            ],
            output='both',
    )

    return LaunchDescription([container])
