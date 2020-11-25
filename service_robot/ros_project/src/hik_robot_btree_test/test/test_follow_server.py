#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import sys
import os
import time
from hik_robot_task.srv import *

def haldle_req(req):
    print "get approach req:", req.cmd, req.num

    return 1

def listener():
    rospy.init_node('test_follow_node')
    s = rospy.Service('HikRobotFollowSrv', HikRobotFollowSrv, haldle_req)
    print('hik robot HikRobotFollowSrv service is ready to get req.')
    rospy.spin()

if __name__ == '__main__':
    listener()

