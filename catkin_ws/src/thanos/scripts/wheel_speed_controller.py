#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64

from numpy import interp


class SpeedController():

    def __init__(self, motor, wmax=12.4, KP=None, KI=None):
        self.motor = motor
        self.wmax = wmax
        self.L = rospy.get_param("~L", 0.146)
        self.r = rospy.get_param("~R", 0.033)
        self.__sample_time = 1.0 / float(rospy.get_param("~RATE", 10.0))
        self.KP = KP or rospy.get_param("~KP", 15.21)
        self.KI = KI or rospy.get_param("~KI", 30.1677)
        self.desired_angular_speed_robot = 0  # velocidade desejada
        self.desired_linear_speed_robot = 0
        self.desired_linear_speed_motor = 0
        self.desired_angular_speed_motor = 0
        self.real_angular_speed_motor = 0  # velocidade lida pelo encoder
        self.real_linear_speed_motor = 0
        self.err = 0
        self.old_err = 0
        self.angular_speed_integrator = []
        self.publisher = rospy.Publisher(
            '/' + self.motor + '_wheel_speed_pwm', Float64,  queue_size=1)

    # obtem a velocidade do topico encoder
    def speed_encoder_callback(self, msg):
        self.real_angular_speed_motor = msg.data
        self.calculate_desired_linear_speed_motor()
        self.calculate_real_linear_speed_motor()
        self.control()

    def set_desired_speeds(self, msg):
        self.desired_linear_speed_robot = msg.linear.x
        self.desired_angular_speed_robot = msg.angular.z
        self.calculate_desired_linear_speed_motor()
        self.calculate_desired_angular_speed_motor()
        self.control()

    def calculate_real_linear_speed_motor(self):
        self.real_linear_speed_motor = self.real_angular_speed_motor * self.r

    def calculate_desired_linear_speed_motor(self):
        if self.motor == "right":
            # v_right = (v + w* L/2)
            self.desired_linear_speed_motor = (
                self.desired_linear_speed_robot + self.desired_angular_speed_robot * self.L / 2.0)
        else:
            #v_left = (-L/2 * w + v)
            self.desired_linear_speed_motor = (
                self.desired_linear_speed_robot - self.desired_angular_speed_robot * self.L / 2.0)

    def calculate_desired_angular_speed_motor(self):
        self.desired_angular_speed_motor = self.desired_linear_speed_motor / self.r

    def control(self):

        self.err = self.desired_angular_speed_motor - self.real_angular_speed_motor

        self.angular_speed_integrator.append(self.KI *
                                             (self.err - self.old_err) * self.__sample_time / 2.0)

        self.angular_speed_proportional = self.KP * self.err
        self.old_err = self.err

        speed = sum(
            self.angular_speed_integrator[-2:]) + self.angular_speed_proportional

        pwm = Float64()

        pwm.data = interp(2*speed, [0, self.wmax], [0, 95])

        # self.publisher.publish(pwm)


# Main
if __name__ == '__main__':
    try:
        rospy.init_node('wheel_speed_controller_node')

        rate = rospy.Rate(rospy.get_param("~RATE", 10))

        left_controller = SpeedController(
            'left', wmax=8, KP=4.44816, KI=4)
        right_controller = SpeedController(
            'right', wmax=8, KP=4.60047, KI=4)

        rospy.Subscriber('left_wheel_encoder', Float64,
                         callback=left_controller.speed_encoder_callback)
        rospy.Subscriber('right_wheel_encoder', Float64,
                         callback=right_controller.speed_encoder_callback)

        rospy.Subscriber('/controlled_cmd_vel', Twist,
                         callback=left_controller.set_desired_speeds)
        rospy.Subscriber('/controlled_cmd_vel', Twist,
                         callback=right_controller.set_desired_speeds)

        while not rospy.is_shutdown():
            
            right_controller.control()
            left_controller.control()

            rate.sleep()

            

    except KeyboardInterrupt:
        destroy()
