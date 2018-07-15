#!/usr/bin/env python
# -*- coding: utf-8 -*-
# pylint: disable=C0321, C0103, C0111, E1101, W0621, I0011

import math
import rospy
import tf
import RPi.GPIO as gpio
import time
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from tf.transformations import euler_from_quaternion


class Controller():

    def __init__(self):
        self.publisher = rospy.Publisher(
            '/controlled_cmd_vel', Twist, queue_size=1)
        self.goal_index = 0
        self.goals = [[1, 0], [1, 1], [0, 1], [0, 0]]
        self.vmax = 0.5
        self.D = rospy.get_param("~D_FEEDBACK", 0.2)
        self.odom = None
        self.theta = None
        self.changing_goal = False
        self.rate = rospy.Rate(rospy.get_param("~RATE", 40.0))

    def sleep_a_lot(self, times=40):
        for i in range(times):
            self.rate.sleep()

    def change_goal(self):
        self.changing_goal = True
        self.publish_zero()
        self.sleep_a_lot()
        self.goal_index += 1
        if self.goal_index > 3:
            self.goal_index = 0
        self.changing_goal = False

    def publish_zero(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.publisher.publish(twist)

    def pose_callback(self, msg):
        if self.changing_goal:
            return self.publish_zero()

        self.odom = msg
        quaternion = (msg.pose.pose.orientation.x,
                      msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z,
                      msg.pose.pose.orientation.w)
        (roll, pitch, yaw) = euler_from_quaternion(quaternion)
        self.theta = yaw
        self.control_action()

    def control_action(self):

        # Feedbacklinearization: [v; w] = A*[vx; vy]
        err_x = self.goals[self.goal_index][0] - self.odom.pose.pose.position.x
        err_y = self.goals[self.goal_index][1] - self.odom.pose.pose.position.y

        norm_error = (err_x**2 + err_y**2) ** (0.5)

        vx = self.vmax * err_x / norm_error
        vy = self.vmax * err_y / norm_error

        if norm_error < 0.25:
            vx = 0
            vy = 0
            print('=========')
            print('change')
            self.change_goal()
            return

        theta = self.theta
        d = self.D
        v = (math.cos(theta) * vx + math.sin(theta) * vy)  # v
        w = (-math.sin(theta) / d * vx + math.cos(theta) / d * vy)  # w

        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w

        self.publisher.publish(twist)


if __name__ == '__main__':
    rospy.init_node('position_controller')
    controller = Controller()

    rospy.Subscriber('odometry_topic', Odometry,
                     controller.pose_callback)

    rospy.spin()
