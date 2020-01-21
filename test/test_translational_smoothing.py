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

import geometry_msgs.msg as geometry_msgs
import nav_msgs.msg as nav_msgs
from numpy import number

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


def number_of_iterations():
    return 10  # 100


def qos_profile():
    return rclpy.qos.qos_profile_sensor_data  # best effort, not latched


class Publisher(object):
    def __init__(self, node_name, qos_profile):
        self.count = 0
        self.stopped = False
        
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
        self.initialise_messages()

    def initialise_messages(self):
        self.cmd_vel = geometry_msgs.Twist()
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.linear.z = 0.0
        self.cmd_vel.angular.x = 0.0
        self.cmd_vel.angular.y = 0.0
        self.cmd_vel.angular.z = 0.0
        self.odom = nav_msgs.Odometry()
        self.odom.header.frame_id = "base_link"
        self.odom.pose.pose.position.x = 0.0
        self.odom.pose.pose.position.y = 0.0
        self.odom.pose.pose.position.z = 0.0
        self.odom.pose.pose.orientation.x = 0.0
        self.odom.pose.pose.orientation.y = 0.0
        self.odom.pose.pose.orientation.z = 0.0
        self.odom.pose.pose.orientation.w = 1.0
        self.odom.pose.covariance[0]  = 0.1
        self.odom.pose.covariance[7]  = 0.1
        self.odom.pose.covariance[35] = 0.2
        self.odom.pose.covariance[14] = 10.0
        self.odom.pose.covariance[21] = 10.0
        self.odom.pose.covariance[28] = 10.0
        self.odom.twist.twist.linear.x = 0.0
        self.odom.twist.twist.linear.y = 0.0
        self.odom.twist.twist.linear.z = 0.0
        self.odom.twist.twist.angular.x = 0.0
        self.odom.twist.twist.angular.y = 0.0
        self.odom.twist.twist.angular.z = 0.0

    def publish(self):
        if self.count < number_of_iterations():
            print("Publishing")
            self.cmd_vel_publisher.publish(self.cmd_vel)
            self.odom_publisher.publish(self.odom)
            self.count += 1
        else:
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

    while not publisher.stopped:
        executor.spin_once(timeout_sec=0.05)

    assert_details("publisher.stopped", "True", publisher.stopped)
    assert(publisher.stopped)

    executor.shutdown()
    publisher.shutdown()
