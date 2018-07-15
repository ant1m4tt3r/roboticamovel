#!/usr/bin/env python
# -*- coding: utf-8 -*-

import RPi.GPIO as gpio
import time
import csv
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

from numpy import interp

# Raspberry board configuration
gpio.setmode(gpio.BOARD)

# Pins PWM,Motor direction (F - forward, B - backward)
motorRight = 31    # Era 33
motorRightF = 18  # Era 31
motorRightB = 16  # Era 29

motorLeft = 29  # Era 32
motorLeftF = 13  # Era 36
motorLeftB = 15  # Era 37

# Robot parameters
vmax1 = 120
vmax2 = 120

# Set pinout motors
gpio.setup(motorRight, gpio.OUT)
gpio.setup(motorRightF, gpio.OUT)
gpio.setup(motorRightB, gpio.OUT)

gpio.setup(motorLeft, gpio.OUT)
gpio.setup(motorLeftB, gpio.OUT)
gpio.setup(motorLeftF, gpio.OUT)

# Define and start PWM
pMotorL = gpio.PWM(motorLeft, 50)
pMotorR = gpio.PWM(motorRight, 50)

pMotorR.start(0)
pMotorL.start(0)

initial_time = time.time()

# Clean raspberry confuguration

pub_left_way = rospy.Publisher('/left_way', Float64, queue_size=1)
pub_right_way = rospy.Publisher('/right_way', Float64, queue_size=1)

home = '/home/ubuntu/catkin_ws/src/thanos/scripts/'
file = open(home + 'pwm.csv', 'w')

right_pwm = 0
left_pwm = 0


def destroy():
    gpio.cleanup()


# Callback from cmd_vel ROS topic
def callback(msg):
    global file, right_pwm, left_pwm, initial_time, pub_left_way, pub_right_way, motorRight, motorRightF, motorRightB, motorLeft, motorLeftF, motorLeftB

    left_way = Float64()
    right_way = Float64()

    l = 0.073
    r = 0.033

    v = msg.linear.x
    w = msg.angular.z

    v_right = (v + w * l) / r
    v_left = (-l * w + v) / r

    pwm_r = interp(v_right, [0, 15], [5, 90])
    pwm_l = interp(v_left, [0, 15], [5, 95])


    # Direction
    if v_right < 0:
        gpio.output(motorRightF, 0)
        gpio.output(motorRightB, 1)
        right_way.data = -1
    else:
        gpio.output(motorRightB, 0)
        gpio.output(motorRightF, 1)
        right_way.data = 1

    if v_left < 0:
        gpio.output(motorLeftF, 0)
        gpio.output(motorLeftB, 1)
        left_way.data = -1
    else:
        gpio.output(motorLeftB, 0)
        gpio.output(motorLeftF, 1)
        left_way.data = 1

    writer = csv.DictWriter(
        file, fieldnames=['time', 'pwm_left', 'pwm_right', 'v', 'w'])

    writer.writerow({'time': time.time() - initial_time,
                     'pwm_left': pwm_l,
                     'pwm_right': pwm_r,
                     'v': v,
                     'w': w})

    file.flush()

    pMotorR.ChangeDutyCycle(abs(pwm_r))
    pMotorL.ChangeDutyCycle(abs(pwm_l))

    pub_left_way.publish(left_way)
    pub_right_way.publish(right_way)


def left_callback(msg):
    global left_pwm
    left_pwm = msg.data


def right_callback(msg):
    global right_pwm
    right_pwm = msg.data


def init():
    global l, r, vmax1, vmax2
    rospy.init_node('motor_node', anonymous=True)
    vmax1 = rospy.get_param("~vmax1", 120)
    vmax2 = rospy.get_param("~vmax2", 120)

    rospy.Subscriber('/controlled_cmd_vel', Twist, callback)
    rospy.Subscriber('/left_wheel_speed_pwm', Float64, left_callback)
    rospy.Subscriber('/right_wheel_speed_pwm', Float64, right_callback)
    rospy.spin()


# Main
if __name__ == '__main__':
    try:
        init()

    except KeyboardInterrupt:
        destroy()

    except:
        print("Other error or exception occurred!")

    finally:
        destroy()
