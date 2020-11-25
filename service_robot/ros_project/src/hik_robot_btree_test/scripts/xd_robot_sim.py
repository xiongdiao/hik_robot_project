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
----------------------------------
Simulate robot main controller!
----------------------------------
req: [group voicenum]

eg:
    0 1 1 

q : to quit
"""

def voicein_srv_handle(req):
    print "get voicein_srv_handle req:"
    print req
    return 1

def handle_patrol_req(req):
    print "get handle_patrol_req req:"
    print req
    return 1

def handle_approach_req(req):
    print "get handle_approach_req req:"
    print req
    return 1

def handle_setpose_req(req):
    print "get handle_setpose_req req:"
    print req
    return 1

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('xd_robot_sim')

    # 初始化 安卓模块->语音服务模块 语音输入请求 set task
    rospy.Service("HikRobotSetTaskSrv", HikRobotSetTaskSrv, voicein_srv_handle)
    rospy.Service('HikRobotPatrolSrv', HikRobotPatrolSrv, handle_patrol_req)
    rospy.Service('HikRobotApproachSrv', HikRobotApproachSrv, handle_approach_req)
    rospy.Service('HikRobotSetManPoseSrv', HikRobotSetManPoseSrv, handle_setpose_req)

    # 初始化 语音服务模块->安卓模块 请求安卓发声
    rospy.wait_for_service('HikRobotVoiceOutSrv')
    voiceout_handle = rospy.ServiceProxy('HikRobotVoiceOutSrv', HikRobotVoiceOutSrv)

    print msg
    param=list()
    req = HikRobotVoiceOutSrvRequest()
    while(1):
        param=[]
        argv = raw_input("req > ")

        if len(argv) < 3:
            if len(argv) > 0 and argv[0] == 'q': 
                break

            print "please input: group voicenum"
            continue

        param = map(int, argv.split())
        print param

        req.group = int(param[0])
        req.voicenum = int(param[1])

        voiceout_handle(req)

        time.sleep(0.5)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


