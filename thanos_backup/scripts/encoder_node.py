#!/usr/bin/env python
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time
import math
import rospy
from std_msgs.msg import String, Float64
import numpy as np

#Define o Raspberry como tipo BOARD
GPIO.setmode(GPIO.BOARD)

#Define os pinos que ser�o utilizados 
def setups():
   
    global PINO_ENCODER
    global PINO_PONTE_H 

    PINO_ENCODER = rospy.get_param("~PINO_ENCODER")
    PINO_PONTE_H = rospy.get_param("~PINO_PONTE_H")
    
    GPIO.setup(PINO_ENCODER, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(PINO_PONTE_H, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

class Encoder():
    
    #Construtor da Classe
    def __init__(self):
        
        self.TOPIC_VEL = rospy.get_param("~TOPIC_VEL")
        self.NUMERO_FAIXAS_ENCODER = rospy.get_param("~NUMERO_FAIXAS_ENCODER") 

        self.__pulsos = 0
        self.__tempo_final = 0
        self.__tempo_inicial = 0
        
        self.velocidade = Float64()

        self.pub_velocidade = rospy.Publisher(self.TOPIC_VEL, Float64, queue_size=1)

    #Função ativada a cada interrupção
    def pulsos_encoder(self,channel):
        self.__pulsos += 1

    #Verifica o sentido de rotação
    def __sentido_rotacao(self):
        sentido = GPIO.input(PINO_PONTE_H)
        if sentido:
            return 1
        else:
            return -1

    #Calcula a velocidade angular com base nos pulsos 
    def calcula_velocidade_angular(self):

        #Lê o instante de tempo em que cada amostra de pulsos começa a ser calculada 
        self.__tempo_final = float(round(time.time() * 1000))

        part_one = 2.0 * math.pi
        
        #Quantidade de pulsos lidos na amostra divido pelo número de faixas do encoder
        part_two = float(self.__pulsos) / float(self.NUMERO_FAIXAS_ENCODER)

        #Período de tempo da amostra
        part_three =(self.__tempo_final - self.__tempo_inicial) / 1000.0
        
        velocidade_angular = part_one * part_two / part_three
        
        self.__pulsos = 0

        #Armazena o instante de tempo do fim do cálculo para ser utilizado na próxima amostra
        self.__tempo_inicial = float(round(time.time() * 1000))

        #Retorna a velocidade com o sinal correspondente ao sentido
        self.velocidade.data = velocidade_angular * self.__sentido_rotacao()

        #Publica a velocidade no tópico
        self.pub_velocidade.publish(self.velocidade)


if __name__ == '__main__':

    try:
        rospy.init_node('encoder_node')
        rate = rospy.Rate(rospy.get_param("~RATE"))

        setups()

        #Gera uma instância da classe Encoder
        encoder = Encoder()

        #Habilita a verificação de interrupção
        GPIO.add_event_detect(PINO_ENCODER, GPIO.FALLING,
                              callback=encoder.pulsos_encoder, bouncetime=1)

        #A cada amostra na frequência "RATE", calcula a velocidade angular
        while not rospy.is_shutdown():
            
            encoder.calcula_velocidade_angular()
            rate.sleep()

    except rospy.ROSInterruptException():
        GPIO.cleanup()       # clean up GPIO on CTRL+C exit
