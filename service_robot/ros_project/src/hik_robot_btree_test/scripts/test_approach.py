#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import sys
import time
from hik_robot_test.srv import * 
from hik_robot_test.msg import *

def voiceout_srv_handle(req):
    print "voiceout_srv_handle"
    return 1

# 模拟安卓收发消息
# HikRobotSetTaskSrv
# HikRobotVoiceOutSrv

def client_test():
    # 语音服务 -> 安卓
    voiceout_srv = rospy.Service("HikRobotVoiceOutSrv", HikRobotVoiceOutSrv, voiceout_srv_handle)

    # 安卓 -> 语音服务
    rospy.wait_for_service('HikRobotSetTaskSrv')
    time.sleep(2)
    try:
        SetTaskSrv = rospy.ServiceProxy('HikRobotSetTaskSrv', HikRobotSetTaskSrv)
        req = HikRobotSetTaskSrvRequest()

        req.group = 2
        req.num = 1
        req.cmd = 1
        req.param = 1
        resp = SetTaskSrv(req)
        print "1", resp.result

        time.sleep(5)
        req.group = 2
        req.num = 1
        req.cmd = 1
        req.param = 1
        resp = SetTaskSrv(req)
        print "2", resp.result

        return resp.result
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



if __name__ == '__main__':
    rospy.init_node('patrol_test')
    client_test()
    rospy.spin()
