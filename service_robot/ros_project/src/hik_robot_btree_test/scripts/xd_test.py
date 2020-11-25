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
Simulate android to Control robot!
----------------------------------
req: [type argv ...]
type:
    patrol      : 0 [cmd num]
    approach    : 1 [cmd angle]
    setpose     : 2 [group room]

eg:
    patrol_start: 0 1 1 
    patrol_stop : 0 0 1 

q : to quit
"""

def voiceout_srv_handle(req):
    print "get voiceout req:", req.group, req.voicenum
    return 1

def file_srv_haldle(req):
    print "get file transfer req:", req.name, req.num
    return 1

def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        pass

    return False


if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('xd_test')

    # 语音服务 -> 安卓 请求发声
    rospy.Service("HikRobotVoiceOutSrv", HikRobotVoiceOutSrv, voiceout_srv_handle)
    rospy.Service('HiRobotVideoFile', HikRobotFileSrv, file_srv_haldle)

    # 安卓 -> 语音服务
    rospy.wait_for_service('HikRobotSetTaskSrv')
    SetTaskSrv = rospy.ServiceProxy('HikRobotSetTaskSrv', HikRobotSetTaskSrv)
    
    # 请求巡检服务
    rospy.wait_for_service('HikRobotPatrolSrv')
    patrol_handle= rospy.ServiceProxy('HikRobotPatrolSrv', HikRobotPatrolSrv)

    # 靠近老人
    rospy.wait_for_service('HikRobotApproachSrv')
    approach_handle= rospy.ServiceProxy('HikRobotApproachSrv', HikRobotApproachSrv)

    # 我在XX
    rospy.wait_for_service('HikRobotSetManPoseSrv')
    setpose_handle= rospy.ServiceProxy('HikRobotSetManPoseSrv', HikRobotSetManPoseSrv)

    print msg
    param=list()
    j = str()
    while(1):
        j=""
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
            req = HikRobotPatrolSrvRequest()
            req.cmd = int(param[1])
            req.num = int(param[2])
            patrol_handle(req)

        elif task_type == 1:
            req = HikRobotApproachSrvRequest()
            req.cmd   = int(param[1])
            req.angle = int(param[2])
            approach_handle(req)

        elif task_type == 2:
            req = HikRobotSetManPoseSrvRequest()
            req.group = int(param[1])
            req.room = int(param[2])
            setpose_handle(req)

        time.sleep(0.5)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

