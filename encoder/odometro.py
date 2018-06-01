# pylint: disable=C0321, C0103, C0111, E1101, W0621, I0011
# encoding: UTF-8

import math
import numpy as np
import rospy

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
    S = (Wr+Wl)*r/2
    return S


#Calcula deslocamento linear
def LinearDesloc(old_V,V,Ts)
    LinDes = Ts* (PrW + W) /2
    return LinDes





