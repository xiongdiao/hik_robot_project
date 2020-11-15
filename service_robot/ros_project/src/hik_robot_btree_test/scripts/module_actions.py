#!/usr/bin/env python
# -*- coding: UTF-8 -*-

'''
初始化需要执行的action

'''

import rospy
import threading
import time

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from hik_robot_test.msg import *
from hik_robot_test.srv import *

class RobotActions():
    def __init__(self, name):
        self.name = name
        rospy.loginfo(self.name + "init")

    def __del__(self):
        rospy.loginfo(self.name + "del")

