#!/usr/bin/env python 
# -*- coding: UTF-8 -*-

import rospy
import sys
import time
from std_msgs.msg import String, Header
from hik_robot_test.msg import HikRobotFile, HikRobotSetTaskMsg
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.pi_trees_lib import *

def SetTaskCb(msg):

    print rospy.get_caller_id(), msg.group, msg.num, msg.cmd, msg.param, msg.angle

    if msg.group == 2:
        if msg.num == 1 and msg.cmd == 1:
            print "康康过来 angle", msg.angle
        elif msg.num == 1 and msg.cmd == 0:
            print "康康别跟了 angle", msg.angle
        elif msg.num == 2 and msg.param == 0:
            print "特定位置巡检 ", msg.goal[0]
        elif msg.num == 2 and msg.param == 1:
            print "全屋巡检", msg.goal
        else:
            print "inviled msg num in group 2"    

    elif msg.group == 3:
        if msg.num == 0:
            print "我在主卧"
        elif msg.num == 1:
            print "我在次卧"
        elif msg.num == 2:
            print "我在阳台"
        elif msg.num == 3:
            print "我在客厅"
        elif msg.num == 4:
            print "我在书房"
        else:
            print "inviled msg num in group 3"    



if __name__ == '__main__':
    rospy.init_node('ProtoalPubTest', anonymous=True)
    rospy.Subscriber("HikRobotSetTaskMsg", HikRobotSetTaskMsg, SetTaskCb)
    print('hik robot task Subscriber is ready to get req.')
    rospy.spin()

