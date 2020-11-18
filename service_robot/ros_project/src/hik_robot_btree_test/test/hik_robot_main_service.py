#!/usr/bin/env python

from __future__ import print_function

import rospy

from hik_robot_test.srv import *

def haldle_set_task_req(req):
    print("get test info [%d %d %d %d]"%(req.group, req.num, req.cmd, req.param), req.goal[0].target_pose.header.stamp)
    #print(req.goal[0].target_pose.header.stamp)

    return req.group + req.num

def set_task_service():
    rospy.init_node('hik_robot_set_task_srv')
    s = rospy.Service('hik_robot_set_task_srv', HikRobotSetTaskSrv, haldle_set_task_req)
    print('hik robot task service is ready to get req.')
    rospy.spin()

if __name__ == "__main__":
    set_task_service()
