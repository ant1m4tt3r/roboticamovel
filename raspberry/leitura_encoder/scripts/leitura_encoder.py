#!/usr/bin/env python
# -*- coding: utf-8 -*-

import RPi.GPIO as GPIO
import time 
import math
import rospy
from std_msgs.msg import String
import numpy as np
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Twist, Pose2D
# from geometry_msgs.msg import Point
# import tf
# from tf.transformations import euler_from_quaternion

#mudar para rospy.Time

GPIO.setmode(GPIO.BOARD)

Pin_encoder = 23
Pin_ponteH = 21

Pulsos = 0

Pulsos_por_volta = 20

raio_roda = 0.032 #em metros
tempo_inicial = 0
tempo_final = 0
rate = rospy.Rate(10)

GPIO.setup(Pin_encoder, GPIO.IN, pull_up_down=GPIO.PUD_UP)  
GPIO.setup(Pin_ponteH, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

def Pulsos_encoder(channel):
     
    Pulsos+=1

def Calcula_velocidade_angular():
    
    if (rospy.get_time() - tempo_inicial >= rate):
        
        tempo_final = rospy.get_time() 
        velocidade_angular = 2*math.pi()*(Pulsos/ Pulsos_por_volta) /(tempo_final - tempo_inicial)
        Pulsos = 0
        tempo_inicial = rospy.get_time() 
      
    return velocidade_angular*Sentido_rotacao()#conferir isso

def Sentido_rotacao():
    
    sentido = GPIO.input(Pin_ponteH)    
    if sentido:
        return 1
    else:
        return -1


GPIO.add_event_detect(Pin_encoder, GPIO.FALLING, callback = Pulsos_encoder, bouncetime= 1)
if __name__ == '__main__':
     
    try:  
        rospy.init_node('leitura_encoder')
        pub = rospy.Publisher('velocidade_encoder', float)
    
        while not rospy.is_shutdown():
            velocidade = Float()
            velocidade.data = Calcula_velocidade_angular() 
            pub.publish(velocidade)
        
        rate.sleep()

    except rospy.ROSInterruptException():
        GPIO.cleanup()       # clean up GPIO on CTRL+C exit  

  