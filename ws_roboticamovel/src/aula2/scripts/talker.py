#!/usr/bin/env python
# -*- coding: utf-8 -*-
##############################

import rospy
from std_msgs.msg import String



if __name__ == '__main__':
  try:
    pub = rospy.Publisher('hello', String, queue_size=1)
    rospy.init_node('talker')
    rate = rospy.Rate(10) #Hz

    i = 0
    while not rospy.is_shutdown():
      aux = String()
      aux.data = 'Hello World ' + str(i)

      pub.publish(aux)

      i += 1
      if i > 1000:
        i = 0

      rate.sleep()

  except rospy.ROSInterruptException():
    pass