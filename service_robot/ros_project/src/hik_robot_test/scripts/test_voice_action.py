#!/usr/bin/env python

import rospy
import time
import actionlib
from hik_robot_test.msg import *

def callback_active():
    print "callback_active"

def callback_feedback(feedback):
    print "callback_feedback", feedback

def callback_done(state, result):
    print "callback_done", result

def action_client():
    voice_ac_client = actionlib.SimpleActionClient('VoiceOutAction', VoiceOutAction)
    voice_ac_client.wait_for_server()

    goal = VoiceOutGoal()
    goal.group = 1
    goal.num = 2
    goal.person = 3
    goal.cmd = 4

    print "start send goal"
    voice_ac_client.send_goal(goal,
                            active_cb = callback_active,
                            feedback_cb = callback_feedback,
                            done_cb = callback_done)

    voice_ac_client.wait_for_result()
    voice_ac_client.get_result()
    print "finish send goal"


if __name__ == '__main__':
    rospy.init_node('voice_action_test')
    action_client()
    #rospy.spin()
