#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import sys
import os
import time
from hik_robot_task.srv import *

def haldle_req(req):
    print "get approach req:", req.cmd, req.angle

    return 1

def listener():
    rospy.init_node('test_node')
    s = rospy.Service('HikRobotApproachSrv', HikRobotApproachSrv, haldle_req)
    print('hik robot HikRobotApproachSrv service is ready to get req.')
    rospy.spin()

if __name__ == '__main__':
    listener()
