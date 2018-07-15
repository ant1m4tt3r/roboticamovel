#!/usr/bin/env python
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time
import math
import rospy
from std_msgs.msg import String, Float64
import numpy as np

# Define o Raspberry como tipo BOARD
GPIO.setmode(GPIO.BOARD)

# Define os pinos que ser�o utilizados


def setups():

    global PINO_ENCODER
    global PINO_PONTE_H

    PINO_ENCODER = rospy.get_param("~PINO_ENCODER")

    GPIO.setup(PINO_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    # GPIO.setup(PINO_PONTE_H, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


class Encoder():

    # Construtor da Classe
    def __init__(self):

        self.TOPIC_VEL = rospy.get_param("~TOPIC_VEL")
        self.NUMERO_FAIXAS_ENCODER = rospy.get_param("~NUMERO_FAIXAS_ENCODER")
        self.FILTER_FACTOR = rospy.get_param("~FILTER_FACTOR", 0.6)

        self.left_way = 1
        self.right_way = 1

        rospy.Subscriber('/left_way', Float64, callback=self.left_way_callback)
        rospy.Subscriber('/right_way', Float64,
                         callback=self.right_way_callback)

        self.__pulsos = 0
        self.__tempo_final = 0
        self.__tempo_inicial = 0

        self.old_velocity = 0

        self.velocidade = Float64()

        self.pub_velocidade = rospy.Publisher(
            self.TOPIC_VEL, Float64, queue_size=1)

    def left_way_callback(self, msg):
        self.left_way = msg.data

    def right_way_callback(self, msg):
        self.right_way = msg.data

    def filter(self, new_velocity):
        velocity = (1 - self.FILTER_FACTOR) * self.old_velocity + \
            self.FILTER_FACTOR * new_velocity

        if abs(velocity) < 0.01:
            return 0

        self.old_velocity = velocity
        return velocity

    # Função ativada a cada interrupção
    def pulsos_encoder(self, channel):
        self.__pulsos += 1

    # Calcula a velocidade angular com base nos pulsos
    def calcula_velocidade_angular(self):

        # Lê o instante de tempo em que cada amostra de pulsos começa a ser
        # calculada
        self.__tempo_final = time.time()

        part_one = math.pi

        # Quantidade de pulsos lidos na amostra divido pelo número de faixas do
        # encoder
        part_two = float(self.__pulsos) / float(self.NUMERO_FAIXAS_ENCODER)

        # Período de tempo da amostra
        part_three = (self.__tempo_final - self.__tempo_inicial)

        velocidade_angular = part_one * part_two / part_three

        self.__pulsos = 0

        # Armazena o instante de tempo do fim do cálculo para ser utilizado na
        # próxima amostra
        self.__tempo_inicial = time.time()

        # Retorna a velocidade com o sinal correspondente ao sentido

        if (self.TOPIC_VEL == 'right_wheel_encoder'):
            self.velocidade.data = self.filter(
                velocidade_angular * 1)
        else:
            self.velocidade.data = self.filter(
                velocidade_angular * 1)


        # Publica a velocidade no tópico
        self.pub_velocidade.publish(self.velocidade)


if __name__ == '__main__':

    try:
        rospy.init_node('encoder_node')
        rate = rospy.Rate(rospy.get_param("~RATE"))

        setups()

        # Gera uma instância da classe Encoder
        encoder = Encoder()

        # Habilita a verificação de interrupção
        GPIO.add_event_detect(PINO_ENCODER, GPIO.RISING,
                              callback=encoder.pulsos_encoder)

        # A cada amostra na frequência "RATE", calcula a velocidade angular
        while not rospy.is_shutdown():

            encoder.calcula_velocidade_angular()
            rate.sleep()

    except rospy.ROSInterruptException():
        GPIO.cleanup()       # clean up GPIO on CTRL+C exit
