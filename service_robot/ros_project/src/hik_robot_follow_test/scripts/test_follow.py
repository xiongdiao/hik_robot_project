#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import sys
from hik_robot_nav_test.msg import *
from hik_robot_nav_test.srv import * 

def talker():
    if  len(sys.argv) != 3 :
        print "need param set msg/srv/action 1/0"
        return -1

    if sys.argv[1] == 'msg':
        pub = rospy.Publisher('HikRobotFollowTaskMsg', HikRobotSetModulesMsg, queue_size=10, tcp_nodelay=True)
        rospy.init_node('test_follow_task', anonymous=True)
        
        if int(sys.argv[2]) == 0:
            print "stop"
            pub.publish(HikRobotSetModulesMsg(0,0,0))
        else:
            print "start" 
            pub.publish(HikRobotSetModulesMsg(0,0,1))

    elif sys.argv[1] == 'srv':
        rospy.wait_for_service('HikRobotFollowTaskSrv')
        try:
            setFollowTaskSrv = rospy.ServiceProxy('HikRobotFollowTaskSrv', HikRobotSetModulesSrv)
            if int(sys.argv[2]) == 0:
                req = HikRobotSetModulesSrvRequest(0,0,0)
            else:
                req = HikRobotSetModulesSrvRequest(0,0,1)

            resp = setFollowTaskSrv(req)
            return resp.result
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    elif sys.argv[1] == 'action':
       pass 

    else:
        print "unknow type"


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
