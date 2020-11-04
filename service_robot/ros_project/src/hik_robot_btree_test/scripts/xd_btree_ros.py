#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from pi_trees_lib.pi_trees_lib import *
import sys


class MsgSubCondition(Task):
    def __init__(self, name, msg_type, msg_condition_cb, tcpNoDelay=True):
        super(MsgSubCondition, self).__init__(name)
        self.name = name

        self.msg_cb = msg_condition_cb
        self.state = False
        rospy.Subscriber(self.name, msg_type, msg_sub_cb)

    def msg_sub_cb(msg_type):
        self.msg_cb(self, msg)


