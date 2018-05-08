#!/usr/bin/env python3
# -*- coding: utf-8 -*- 
################################################

import rospy
from std_msgs.msg import String
import numpy as np
from nav_msgs.msg import Odometry



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

# Adiciona ruído gaussiano
def add_noise(value, gain=0.3, t = 1):
    global count    
    if t:
        count += 1
        if count > 3:
            noise = gain*np.random.normal(0,1,1)
            value += noise.item() 
            count = 0
    else:
        noise = gain*np.random.normal(0,1,1)
        value += noise.item() 
    return value

#######################################################################
# Callback para a leitura da posição
def callback_pose(msg):
    global pose_robot, pose_robot_noisy, USING_NOISE, GAINX, GAINY
    
    if USING_NOISE:
        pose_robot_noisy = msg
        pose_robot_noisy.pose.pose.position.x = add_noise(msg.pose.pose.position.x, GAINX)   
        pose_robot_noisy.pose.pose.position.y = add_noise(msg.pose.pose.position.y, GAINY,0)  
    
    pose_robot = msg    
    
  
#######################################################################  



#######################################################################
# MAIN MODULE  
if __name__ == '__main__':
    try:    
        
        rospy.init_node('noise')
        rate = rospy.Rate(rospy.get_param("~RATE", 30)) 
        
        
        
        #variáveis globais
        DEBUG = rospy.get_param("~DEBUG", 0)
        USING_NOISE = rospy.get_param("~USING_NOISE", 0)
        TOPIC_ODOM_SUB = rospy.get_param("~TOPIC_ODOM", '/robot_0/odom')
        TOPIC_ODOM_PUB = '/odom'
        GAINX = rospy.get_param("~GAINX", 0.5)
        GAINY = rospy.get_param("~GAINY", 0.5)
        
        pose_robot = Odometry()
        pose_robot.pose.pose.orientation.w = 1
        pose_robot_noisy = Odometry()
        pose_robot_noisy.pose.pose.orientation.w = 1
        
        rospy.Subscriber(TOPIC_ODOM_SUB,Odometry,callback_pose,queue_size=1)
        pub = rospy.Publisher(TOPIC_ODOM_PUB,Odometry,queue_size=1)
            
        count = 0
        
        
        
        while not rospy.is_shutdown():
            
            if USING_NOISE:
                pub.publish(pose_robot_noisy)
            else:
                pub.publish(pose_robot)
            
            rate.sleep()
            
            
            
            
    
    
    except rospy.ROSInterruptException():
        pass