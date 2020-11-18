#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import time
from geometry_msgs.msg import Twist
import sys, select, termios, tty

from hik_robot_btree_test.msg import *
from hik_robot_btree_test.srv import *
from hik_robot_task.msg import *
from hik_robot_task.srv import *

msg = """
Simulate android to Control robot!
---------------------------
req: [group num cmd param angle goal[]]
q : to quit
"""

def voiceout_srv_handle(req):
    print "get voiceout req:", req.group, req.voicenum
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
        
        argv = raw_input("req > ")
        if len(argv) < 7:
            if len(argv) > 0 and argv[0] == 'q': 
                break

            print "please input: argv num cmd param"
            continue

        req.group = int(argv[0])
        req.num   = int(argv[2])
        req.cmd   = int(argv[4])
        req.param = int(argv[6])
        

        print "send req :", req.group, req.num, req.cmd, req.param
        SetTaskSrv(req)
        time.sleep(0.5)


    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

