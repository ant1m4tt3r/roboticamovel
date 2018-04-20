#!/usr/bin/env python
# -*- coding: utf-8 -*-
##############################

import rospy
from std_msgs.msg import String

def callback(msg):
  print(msg.data)
    
if __name__ == '__main__':
  try:
    rospy.init_node('listener')

    rospy.Subscriber('hello', String, callback)

    while not rospy.is_shutdown():
      pass

  except rospy.ROSInterruptException():
    pass