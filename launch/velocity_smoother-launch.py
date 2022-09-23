#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#
# Copyright (c) 2020 Open Source Robotics Foundation, Inc.
# 
# Software License Agreement (BSD License 2.0)
#   https://raw.githubusercontent.com/kobuki-base/velocity_smoother/license/LICENSE

"""Launch the velocity smoother node with default configuration."""

import os

import ament_index_python.packages
import launch
import launch_ros.actions

import yaml

def generate_launch_description():
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
    velocity_smoother_node = launch_ros.actions.Node(
        package='velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='both',
        parameters=[params])

    return launch.LaunchDescription([velocity_smoother_node])
