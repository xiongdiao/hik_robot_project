#!/usr/bin/env python 
# -*- coding: UTF-8 -*-


import rospy
import sys
import signal
import time
import subprocess
from std_msgs.msg import String, Header
from hik_robot_task.srv import *


def client_req():
    rospy.wait_for_service('HikRobotSetManPoseSrv')
    SetposeProxy = rospy.ServiceProxy('HikRobotSetManPoseSrv', HikRobotSetManPoseSrv)
    req = HikRobotSetManPoseSrvRequest()
    req.group = 2
    req.room = 7
    resp = SetposeProxy(req)
    return resp.result

if __name__ == "__main__":
    print("get service resp %d "%client_req())

