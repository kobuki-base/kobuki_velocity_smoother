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

import enum

import rclpy
import rclpy.executors

import geometry_msgs.msg as geometry_msgs
import nav_msgs.msg as nav_msgs

##############################################################################
# Helpers
##############################################################################


def banner(msg):
    print("\n" + 80 * "*")
    print("* " + msg.center(80))
    print(80 * "*" + "\n")

def assert_banner():
    print("----- Asserts -----")


def assert_details(text, expected, result):
    print(text +
          "." * (40 - len(text)) +
          "{}".format(expected) +
          " [{}]".format(result)
    )


def setup_module(module):
    banner("ROS Init")
    rclpy.init()


def teardown_module(module):
    banner("ROS Shutdown")
    rclpy.shutdown()


def timeout():
    return 0.3

def qos_profile():
    return rclpy.qos.qos_profile_sensor_data  # best effort, not latched

##############################################################################
# Classes
##############################################################################

class Parameters(object):
    def __init__(self):
        self.velocity_maximum = 0.50
        self.ramp_increment = 0.02
        self.ramp_decrement = 0.02

class Command(object):
    def __init__(self):
        self.cmd_vel, self.odom = self.initialise_messages()
        self.profile = self.generate_profile()

    @staticmethod
    def generate_profile():
        parameters = Parameters()
        x_vel = 0.0
        profile = []
        while x_vel <= parameters.velocity_maximum:  # RAMP_UP
            profile.append(x_vel)
            x_vel += parameters.ramp_increment
        count = 0
        interval_length = len(profile)
        while count < interval_length:  # RAMP_LEVEL
            profile.append(x_vel)
            count += 1
        while x_vel > 0.0:  # RAMP_DOWN
            profile.append(x_vel)
            x_vel -= parameters.ramp_decrement
            x_vel = 0.0 if x_vel < 0.0 else x_vel
        count = 0
        while count < interval_length:  # ZERO
            profile.append(x_vel)
            count += 1
        count = 0
        x_vel = parameters.velocity_maximum
        while count < interval_length:  # UP
            profile.append(x_vel)
            count += 1
        count = 0
        x_vel = 0.0
        while count < interval_length:  # DOWN
            profile.append(x_vel)
            count += 1
        return profile


    @staticmethod
    def initialise_messages():
        cmd_vel = geometry_msgs.Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        cmd_vel.angular.x = 0.0
        cmd_vel.angular.y = 0.0
        cmd_vel.angular.z = 0.0
        odom = nav_msgs.Odometry()
        odom.header.frame_id = "base_link"
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        odom.pose.covariance[0]  = 0.1
        odom.pose.covariance[7]  = 0.1
        odom.pose.covariance[35] = 0.2
        odom.pose.covariance[14] = 10.0
        odom.pose.covariance[21] = 10.0
        odom.pose.covariance[28] = 10.0
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0
        return cmd_vel, odom

class Publisher(object):
    def __init__(self, node_name, qos_profile):
        self.stopped = False
        self.command = Command()

        self.node = rclpy.create_node(node_name)
        self.cmd_vel_publisher = self.node.create_publisher(
            msg_type=geometry_msgs.Twist,
            topic="~/cmd_vel",
            qos_profile=qos_profile
        )
        self.odom_publisher = self.node.create_publisher(
            msg_type=nav_msgs.Odometry,
            topic="~/odom",
            qos_profile=qos_profile
        )
        self.timer = self.node.create_timer(
            timer_period_sec=0.1,
            callback=self.publish
        )

    def publish(self):
        try:
            commanded_velocity = self.command.profile.pop(0)
            print("Publishing .... [{:0.2f}]".format(commanded_velocity))
            self.command.cmd_vel.linear.x = commanded_velocity
            self.command.odom.twist.twist.linear.x = commanded_velocity
            self.cmd_vel_publisher.publish(self.command.cmd_vel)
            self.odom_publisher.publish(self.command.odom)
        except IndexError:
            self.timer.cancel()
            self.stopped = True

    def shutdown(self):
        self.cmd_vel_publisher.destroy()
        self.odom_publisher.destroy()
        self.node.destroy_timer(self.timer)
        self.node.destroy_node()

##############################################################################
# Tests
##############################################################################


def test_publishing():
    banner("Publishing")

    publisher = Publisher(
        node_name="publisher",
        qos_profile=qos_profile()
    )

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(publisher.node)

    assert_banner()

    try:
        while not publisher.stopped:
            executor.spin_once(timeout_sec=0.05)

        assert_details("publisher.stopped", "True", publisher.stopped)
        assert(publisher.stopped)
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    publisher.shutdown()
