#!/usr/bin/env python3
# -*- coding: utf-8 -*- 
################################################

import rospy
from std_msgs.msg import String
from turtlesim.msg import Pose
import numpy as np


#######################################################################
# FUNCTIONS

#######################################################################  
#print if it's in DEBUG mode
def printlog(s, type=0):
  global DEBUG
  if DEBUG:
    if(type == 0): #info mensage
      rospy.loginfo(s)
    elif type == 1: #warning mensage
      rospy.logwarn(s) 
    elif type == 2:
      rospy.logerr(s) # error mensage
#######################################################################

#######################################################################
# Callback para a leitura da posição da turtle1
#mensagem turtlesim/Pose contém:
    #float32 x
    #float32 y
    #float32 theta
    #float32 linear_velocity
    #float32 angular_velocity
def callback_poseturtle1(msg):    
    printlog("######\n turtle1 está na posição x={}, y={} \n com a orientação = {}".format(msg.x,msg.y, msg.theta))
  
#######################################################################  


#######################################################################
# Converte uma velocidade em vx e vy do sistema inercial dada a orientação do robô
# para velocidade linear em x (v) e velocidade angular (w) do sistema do robô
def converte_velocidade(vx, vy, theta):
    #Feedbacklinearization: [v; w] = A*[vx; vy]    
    d = 1
    v =  (np.cos(theta)*vx + np.sin(theta)*vy) #v
    w = (-np.sin(theta)/d*vx + np.cos(theta)/d*vy) #w 
    
    printlog("(v, w) = ({},{})".format(v,w))
    return v,w
#######################################################################

#######################################################################
# Controle Proporcional de posição
def controleP(p_alvo, p_atual, kp=2, v_max = 1):     
    vx = 0
    vy = 0
    printlog("(vx, vy) = ({},{})".format(vx,vy))
    return vx, vy
#######################################################################



#######################################################################
# MAIN MODULE  
if __name__ == '__main__':
    try:    
        
        rospy.init_node('track')
        rate = rospy.Rate(10) #Hz
        
        
        #variáveis globais
        DEBUG = True        
        pose_turtle1 = Pose()
        
        
        rospy.Subscriber('/turtle1/pose',Pose,callback_poseturtle1,queue_size=1)
        
        while not rospy.is_shutdown():
            pass
    
    
    
    except rospy.ROSInterruptException():
        pass