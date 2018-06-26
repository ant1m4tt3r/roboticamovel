#!/usr/bin/env python
# -*- coding: utf-8 -*-

import RPi.GPIO as gpio
import time

import rospy
from geometry_msgs.msg import Twist

# Raspberry board configuration
gpio.setmode(gpio.BOARD)

# Pins PWM,Motor direction (F - forward, B - backward)
motorRight = 31    # Era 33
motorRightF = 18  # Era 31 
motorRightB = 16 # Era 29

motorLeft = 29  # Era 32
motorLeftF = 13  # Era 36
motorLeftB = 15  # Era 37

# Robot parameters
l = 7.3
r = 3.3
vmax1 = 120
vmax1 = 120

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

# Clean raspberry confuguration


def destroy():
    gpio.cleanup()


# Callback from cmd_vel ROS topic
def callback(msg):
    global l, r, vmax1, vmax2, motorRight, motorRightF, motorRightB, motorLeft, motorLeftF, motorLeftB

    # Right
    v1 = (msg.linear.x + msg.angular.z * l) / r
    # Left
    v2 = (-l * msg.angular.z + msg.linear.x) / r
    # Vel -> PWM
    v1 = 100 * v1 / vmax1
    v2 = 100 * v2 / vmax2
    # Direction
    if v1 < 0:
        gpio.output(motorRightF, 0)
        gpio.output(motorRightB, 1)
        v1 = max(v1, -100)
    else:
        gpio.output(motorRightB, 0)
        gpio.output(motorRightF, 1)
        v1 = min(v1, 100)

    if v2 < 0:
        gpio.output(motorLeftF, 0)
        gpio.output(motorLeftB, 1)
        v2 = max(v2, -100)
    else:
        gpio.output(motorLeftB, 0)
        gpio.output(motorLeftF, 1)
        v2 = min(v2, 100)
    # Change PWM dutycycle

    if v2 >100: v2 = 100
    if v1 > 100: v1 = 100

    pMotorL.ChangeDutyCycle(abs(v2))
    pMotorR.ChangeDutyCycle(abs(v1))

# Define ROS node


def noRodas():
    global l, r, vmax1, vmax2
    rospy.init_node('controleRodas', anonymous=True)
    vmax1 = rospy.get_param("~vmax1", 120)
    vmax2 = rospy.get_param("~vmax2", 120)
    r = rospy.get_param("~r", 3)
    l = rospy.get_param("~l", 7)

    rospy.Subscriber('/cmd_vel', Twist, callback)
    rospy.spin()


# Main
if __name__ == '__main__':
    try:
        noRodas()

    except KeyboardInterrupt:
        destroy()

    except:
        print("Other error or exception occurred!")

    finally:
        destroy()
