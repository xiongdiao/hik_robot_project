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
from hik_robot_task.srv import *

def haldle_req(req):
    print "get video data"
    data = req.data
    name = req.name
    with open("../" + name, 'wb') as f:
        f.write(data)

    return 1

def listener():
    rospy.init_node('hik_robot_set_task_srv')
    s = rospy.Service('HiRobotVideoFile', HikRobotFileSrv, haldle_req)
    print('hik robot task service is ready to get req.')
    rospy.spin()

if __name__ == '__main__':
    listener()
