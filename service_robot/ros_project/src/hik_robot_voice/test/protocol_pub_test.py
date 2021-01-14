#!/usr/bin/env python 
# -*- coding: UTF-8 -*-

import rospy
import sys
import time
from std_msgs.msg import String, Header
from hik_robot_test.msg import HikRobotFile, HikRobotSetTaskMsg
from rosjava_hikrobot_msgs.msg import HikRobotVoiceOut
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.pi_trees_lib import *

def pubFileTest():
    video_pub= rospy.Publisher('HikRobotVideoFile', HikRobotFile, queue_size=10)
    time.sleep(1)
    with open('./source.mp4', 'rb') as f:
        video_file = HikRobotFile()
        video_file.name = "1.mp4"
        video_file.data = f.read()
        video_pub.publish(video_file)
        print "传送视频文件完成"


def pubVoiceOutTest():
    voice_pub = rospy.Publisher('HikRobotVoiceOut', HikRobotVoiceOut, queue_size=10)
    time.sleep(1)
    # 你在哪
    print "你在哪 HikRobotVoiceOut [1, 0]"
    voice_pub.publish(HikRobotVoiceOut(1, 0, 0, 0))
    time.sleep(2)

    print "靠近老人已完成 HikRobotVoiceOut [4, 0]"
    voice_pub.publish(HikRobotVoiceOut(4, 0, 0, 0))
    time.sleep(2)

if __name__ == '__main__':
    rospy.init_node('pubTest', anonymous=True)
    pubVoiceOutTest()
    pubFileTest()
    rospy.spin()



