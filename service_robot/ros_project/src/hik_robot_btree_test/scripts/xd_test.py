#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import time
from geometry_msgs.msg import Twist
import sys, select, termios, tty

from hik_robot_test.srv import * 
from hik_robot_test.msg import *

msg = """
Simulate android to Control robot!
---------------------------
req: [group num cmd param angle goal[]]
q : to quit
"""

SetTaskBindings = {
        '1':(2, 2, 1, 1),
        '2':(2, 1, 1, 1),
        '3':(0, 0, 0, 1),
        '4':(0, 0, 0, 1),
        '5':(0, 0, 0, 1),
        '6':(0, 0, 0, 1),
        '7':(0, 0, 0, 1),
        '8':(0, 0, 0, 1),
        }

def voiceout_srv_handle(req):
    print "get voiceout req:", req.group, req.num, req.person, req.cmd
    return 1

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('xd_test')

    # 安卓 -> 语音服务
    rospy.wait_for_service('HikRobotSetTaskSrv')
    SetTaskSrv = rospy.ServiceProxy('HikRobotSetTaskSrv', HikRobotSetTaskSrv)
    voiceout_srv = rospy.Service("HikRobotVoiceOutSrv", HikRobotVoiceOutSrv, voiceout_srv_handle)
    req = HikRobotSetTaskSrvRequest()

    print msg
    while(1):
        
        group = raw_input("req > ")
        if len(group) < 7:
            if len(group) > 0 and group[0] == 'q': 
                break

            print "please input: group num cmd param"
            continue

        req.group = int(group[0])
        req.num   = int(group[2])
        req.cmd   = int(group[4])
        req.param = int(group[6])
        

        print "send req :", req.group, req.num, req.cmd, req.param
        SetTaskSrv(req)
        time.sleep(0.5)


    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

