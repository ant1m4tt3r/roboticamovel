#!/usr/bin/env python
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time
import math
import rospy
from std_msgs.msg import String, Float64
import numpy as np
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist, Pose2D
# from geometry_msgs.msg import Point
# import tf
# from tf.transformations import euler_from_quaternion

# mudar para rospy.Time

GPIO.setmode(GPIO.BOARD)

pin_encoder = 7
pin_ponte_h = 13

pulsos = 0

pulsos_por_volta = 20

raio_roda = 0.032  # em metros
tempo_inicial = 0.0
tempo_final = 0.0

GPIO.setup(pin_encoder, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(pin_ponte_h, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


def pulsos_encoder(channel):
    global pulsos
    pulsos += 1


def calcula_velocidade_angular():
    global tempo_final
    global tempo_inicial
    global pulsos
    global pulsos_por_volta

    tempo_final = float(round(time.time() * 1000))

    part_one = 2.0 * math.pi
    part_two = float(pulsos) / float(pulsos_por_volta)
    part_three = (tempo_final - tempo_inicial) / 1000.0

    velocidade_angular = part_one * part_two / part_three

    pulsos = 0

    tempo_inicial = float(round(time.time() * 1000))
    return velocidade_angular * sentido_rotacao()  # conferir isso


def sentido_rotacao():
    sentido = GPIO.input(pin_ponte_h)
    if sentido:
        return 1
    else:
        return -1


if __name__ == '__main__':

    try:
        rospy.init_node('leitura_encoder')
        rate = rospy.Rate(10)
        pub = rospy.Publisher('velocidade_encoder', Float64, queue_size=1)

        GPIO.add_event_detect(pin_encoder, GPIO.FALLING,
                              callback=pulsos_encoder, bouncetime=1)

        while not rospy.is_shutdown():
            velocidade = Float64()
            velocidade.data = calcula_velocidade_angular()
            pub.publish(velocidade)
            rate.sleep()

    except rospy.ROSInterruptException():
        GPIO.cleanup()       # clean up GPIO on CTRL+C exit
