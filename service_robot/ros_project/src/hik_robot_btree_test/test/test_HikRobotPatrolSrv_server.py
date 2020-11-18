#!/usr/bin/env python
# -*- coding: UTF-8 -*-



import rospy
import sys
import os
import time

from hik_robot_task.srv import *

def haldle_req(req):
    print "get req: ", req.cmd, req.num
    return 1

def listener():
    rospy.init_node('hik_robot_set_task_srv')
    s = rospy.Service('HikRobotPatrolSrv', HikRobotPatrolSrv, haldle_req)
    print('hik robot task service is ready to get req.')
    rospy.spin()

if __name__ == '__main__':
    listener()

