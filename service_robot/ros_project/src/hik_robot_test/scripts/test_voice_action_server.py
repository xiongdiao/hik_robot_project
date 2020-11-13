#!/usr/bin/env python

import sys
import rospy
import time
from hik_robot_test.srv import * 
from hik_robot_test.msg import *
import actionlib


class VoiceAction(object):
    _feedback = VoiceOutFeedback()
    _result = VoiceOutResult()

    def __init__(self, name):
        self.name = name
        self._as = actionlib.SimpleActionServer("VoiceOutAction", VoiceOutAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        print "execute_cb start"

        self._feedback.feedback = 1
        self._as.publish_feedback(self._feedback)
        time.sleep(0.5)


        self._result.group = 1
        self._result.num = 2
        self._as.set_succeeded(self._result)

        print "execute_cb finish"
        


if __name__ == '__main__':
    rospy.init_node('voice_action_server')

    voiceAction = VoiceAction("voice_action")

    rospy.spin()
