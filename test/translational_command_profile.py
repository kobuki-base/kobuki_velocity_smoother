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
        self.generator = self.generate_profile()
        self.profile = []

    def generate_profile(self):
        parameters = Parameters()
        x_vel = 0.0
        # RAMP_UP
        while x_vel <= parameters.velocity_maximum:
            self.profile.append(x_vel)
            yield x_vel
            x_vel += parameters.ramp_increment
        # RAMP_LEVEL
        count = 0
        interval_length = len(self.profile)
        while count < interval_length:
            self.profile.append(x_vel)
            yield x_vel
            count += 1
        # RAMP_DOWN
        while x_vel > 0.0:
            self.profile.append(x_vel)
            yield x_vel
            x_vel -= parameters.ramp_decrement
            x_vel = 0.0 if x_vel < 0.0 else x_vel
        # ZERO
        count = 0
        while count < interval_length:
            self.profile.append(x_vel)
            yield x_vel
            count += 1
        # UP
        count = 0
        x_vel = parameters.velocity_maximum
        while count < interval_length:
            self.profile.append(x_vel)
            yield x_vel
            count += 1
        # DOWN
        count = 0
        x_vel = 0.0
        while count < interval_length:
            self.profile.append(x_vel)
            yield x_vel
            count += 1


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
            commanded_velocity = next(self.command.generator)
            print("Publishing .... [{:0.2f}]".format(commanded_velocity))
            self.command.cmd_vel.linear.x = commanded_velocity
            self.command.odom.twist.twist.linear.x = commanded_velocity
            self.cmd_vel_publisher.publish(self.command.cmd_vel)
            self.odom_publisher.publish(self.command.odom)
        except StopIteration:
            print("PROFILE_SENT")
            self.timer.cancel()
            self.stopped = True

    def shutdown(self):
        self.cmd_vel_publisher.destroy()
        self.odom_publisher.destroy()
        self.node.destroy_timer(self.timer)
        self.node.destroy_node()

##############################################################################
# Main
##############################################################################


if __name__== "__main__":
    banner("Translational Profiling")

    rclpy.init()

    publisher = Publisher(
        node_name="publisher",
        qos_profile=qos_profile()
    )

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(publisher.node)

    try:
        while not publisher.stopped:
            executor.spin_once(timeout_sec=0.05)
    except KeyboardInterrupt:
        pass

    executor.shutdown()
    publisher.shutdown()
    rclpy.shutdown()
