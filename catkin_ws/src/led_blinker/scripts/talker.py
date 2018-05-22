#!/usr/bin/env python
# -*- coding: utf-8 -*- 
################################################

import rospy
from std_msgs.msg import String

#######################################################################
# MAIN MODULE  
if __name__ == '__main__':
    try:    
        pub = rospy.Publisher('/blink',String,queue_size=1)
        rospy.init_node('talker')
        rate = rospy.Rate(3) #Hz
        num = True
        while not rospy.is_shutdown():

            aux = String()

            if num == True:
                aux.data = "ON"
                pub.publish(aux)
                rate.sleep()
                num = not num
            else:
                aux.data = "OFF"
                pub.publish(aux)
                rate.sleep()
                num = not num

            
           # aux.data = "ON"
           # pub.publish(aux)      
           # rate.sleep()
           # aux.data = "OFF"
           # pub.publish(aux)
           # rate.sleep()

    except rospy.ROSInterruptException():
        pass