#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy

from std_msgs.msg import String, Header
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from hik_robot_test.srv import *

def client_req():
    rospy.wait_for_service('HikRobotVoiceOutSrv')
    try:
        VoiceOutProxy = rospy.ServiceProxy('HikRobotVoiceOutSrv', HikRobotVoiceOutSrv)
        req = HikRobotVoiceOutSrvRequest()
        req.group = 1
        req.num = 2
        req.cmd = 3
        req.person = 4
        resp = VoiceOutProxy(req)
        return resp.result

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



def usage():
    return "%s [x y]"%sys.argv[0]


if __name__ == "__main__":
    print("get service resp %d "%client_req())

