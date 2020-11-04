#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from std_msgs.msg import String 
from rosjava_hikrobot_msgs.msg import HikRobotVoiceOut


def voiceOutSubCb(msg):
    group = msg.group
    num   = msg.num
    person= msg.person
    cmd   = msg.cmd

    print rospy.get_caller_id(), group, num, person, cmd

def listener():
    rospy.init_node('androidSimulate', anonymous=True)
    rospy.Subscriber("HikRobotVoiceOut", HikRobotVoiceOut, voiceOutSubCb)
    print('hik robot android Subscriber is ready to get req.')
    rospy.spin()

if __name__ == '__main__':
    listener()
