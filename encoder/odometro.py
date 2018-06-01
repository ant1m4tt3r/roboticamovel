#!/usr/bin/env python
# pylint: disable=C0321, C0103, C0111, E1101, W0621, I0011
# encoding: UTF-8

import math
import numpy as np
import rospy

#variáveis globais
def setups()
    global right_speed
    global left_speed
    global desloc
    global lin_speed
    global ang_speed
    global sample_time
    global wheels_radius
    global wheels_dist

    lin_speed = [0]
    ang_speed = [0]
    desloc = 0
    sample_time = rospy.get_param("~TEMPO_DE_AMOSTRAGEM")
    wheels_radius = rospy.get_param("~RAIO_DAS_RODAS")
    wheels_dist = rospy.get_param("~DIST_ENTRE_RODAS",1)



#calcula a velocidade angular
#Parâmetros
#   Wr = velocidade angular da roda direita
#   Wl = velocidade angular da roda esquerda
#   r  = Raio das rodas
#   L  = Distância entre as rodas
def AngSpeed(Wr,Wl,r,L)
    W = (Wr+Wl)*r/L
    return W


#calcula a velocidade linear
#Parâmetros
#   Wr = velocidade angular da roda direita
#   Wl = velocidade angular da roda esquerda
#   r  = Raio das rodas
def LinSpeed(Wr,Wl,r)
    S = (Wr+Wl)*r/2.0
    return S


#Calcula deslocamento linear
def LinearDesloc(old_v,v,ts)
    LinDes = ts* (old_v + v) /2
    return LinDes

def __encoder_right_msg(msg)
    right_speed = msg.data
    rospy.loginfo(msg.data)

def __encoder_left_msg(msg)
    left_speed = msg.data
    rospy.loginfo(msg.data)
    
if __name__ == '__main__':
    try:
        rospy.init_node('odometro')
        setups()
        while not rospy.is_shutdown():
            encoder_right = rospy.Subscriber('leitura_encoder_direito',Float64(),__encoder_right_msg)
            encoder_left  = rospy.Subscriber('leitura_encoder_esquerdo',Float64(),__encoder_left_msg)
            ang_speed.append(AngSpeed(right_speed,left_speed,wheels_radius,wheels_dist))
            lin_speed.append(LinSpeed(right_speed,left_speed,wheels_radius))
            desloc =  desloc + LinearDesloc(lin_speed[-2],lin_speed[-1],sample_time)
       
    except rospy.ROSInterruptException()
        pass

