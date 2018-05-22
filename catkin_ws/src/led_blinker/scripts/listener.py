#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import String
import time

LED_PIN = 11

def setup():
    global LED_PIN
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(LED_PIN, GPIO.OUT)
    GPIO.output(LED_PIN, GPIO.HIGH)


def destroy():
    global LED_PIN
    GPIO.output(LED_PIN, GPIO.LOW)
    GPIO.cleanup()


def led(state):
    global LED_PIN
    if state:
        GPIO.output(LED_PIN, GPIO.HIGH)
    else:
        GPIO.output(LED_PIN, GPIO.LOW)


def callback(msg):
    print(msg)
    if msg.data == 'ON':
        led(True)
    else:
        led(False)


if __name__ == '__main__':
    try:

        setup()

        rospy.init_node('listener')
        rate = rospy.Rate(1)  # Hz

        rospy.Subscriber('/blink', String,
                         callback, queue_size=1)

        while not rospy.is_shutdown():
            rate.sleep()

        destroy()

    except rospy.ROSInterruptException():
        destroy()
