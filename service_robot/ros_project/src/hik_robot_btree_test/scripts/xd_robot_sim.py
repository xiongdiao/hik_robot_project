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
type:
    voice out : 0 [cmd voicenum]
    file out  : 1 [name data]

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

def handle_follow_req(req):
    print "get handle_follow_req req:"
    print req
    return 1

def handle_nav_req(req):
    print "get handle_nav_req req:"
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
    rospy.Service('HikRobotNavSrv', HikRobotNavSrv, handle_nav_req)
    rospy.Service('HikRobotFollowSrv', HikRobotFollowSrv, handle_follow_req)

    # 初始化 语音服务模块->安卓模块 请求安卓发声
    rospy.wait_for_service('HikRobotVoiceOutSrv')
    voiceout_handle = rospy.ServiceProxy('HikRobotVoiceOutSrv', HikRobotVoiceOutSrv)

    # 初始化 文件上传模块
    rospy.wait_for_service('HiRobotVideoFile')
    file_client_handle = rospy.ServiceProxy('HiRobotVideoFile', HikRobotFileSrv)

    print msg
    param=list()
    while(1):
        param=[]
        argv = raw_input("req > ")

        if len(argv) < 5:
            if len(argv) > 0 and argv[0] == 'q': 
                break
            print "please input: task_type argv ..."
            continue

        param = map(int, argv.split())
        print param

        task_type = int(param[0])
        if task_type == 0:
            req = HikRobotVoiceOutSrvRequest()
            req.group = int(param[1])
            req.voicenum = int(param[2])
            voiceout_handle(req)

        elif task_type == 1:
            file_req = HikRobotFileSrvRequest()
            file_req.name = str(param[1])
            file_client_handle(file_req)

        time.sleep(0.5)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


