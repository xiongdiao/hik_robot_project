#!/usr/bin/env python

from __future__ import print_function

import rospy

from hik_robot_test.srv import *

def haldle_set_task_req(req):
    print("get test info [%d %d %d %d]"%(req.group, req.num, req.person, req.cmd))

    return req.group + req.num

def set_task_service():
    rospy.init_node('robot_voice_out_srv')
    s = rospy.Service('HikRobotVoiceOutSrv', HikRobotVoiceOutSrv, haldle_set_task_req)
    print('hik robot task service is ready to get req.')
    rospy.spin()

if __name__ == "__main__":
    set_task_service()

