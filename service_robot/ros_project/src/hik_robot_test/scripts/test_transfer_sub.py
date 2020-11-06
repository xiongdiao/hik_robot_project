#!/usr/bin/env python
# -*- coding: UTF-8 -*-



#with open('source.mp4', 'rb') as f:
#    data = f.read()
#    with open('dst.mp4', 'wb') as f:
#        f.write(data)

import rospy
import sys
import os
import time
from hik_robot_test.msg import HikRobotFile

def set_task_callback(msg):
    print "get video data"
    data = msg.data
    name = msg.name
    with open(name, 'wb') as f:
        f.write(data)

def listener():
    rospy.init_node('VideoFileSub', anonymous=True)
    rospy.Subscriber("HiRobotVideoFile", HikRobotFile, set_task_callback)
    print('hik robot task Subscriber is ready to get req.')
    rospy.spin()

if __name__ == '__main__':
    listener()
