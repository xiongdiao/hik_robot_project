#!/usr/bin/env python 
# -*- coding: UTF-8 -*-


import rospy
import sys
import time
from std_msgs.msg import String, Header
from hik_robot_test.msg import HikRobotFile
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.pi_trees_lib import *


class PubVideoTask(Task):
    def __init__(self, name, path, dst_video_name, *args, **kwargs):
        super(PubVideoTask, self).__init__(name, *args, **kwargs)
        self.name = name
        self.status = None
        self.video_path = path
        self.video_file = HikRobotFile()
        self.video_file.name = dst_video_name 
        self.video_pub = rospy.Publisher('HiRobotVideoFile', HikRobotFile, queue_size=10)
        print self.name, "init"

    def run(self):
        self.status = TaskStatus.FAILURE
        print self.name, "run"
        with open(self.video_path, 'rb') as f:
            data = f.read()
            self.video_file.data = data
            self.video_pub.publish(self.video_file)
            self.status = TaskStatus.SUCCESS

        return self.status

if __name__ == '__main__':
    rospy.init_node('VideoFilePub', anonymous=True)
    task = PubVideoTask("PubVideoTask", './source.mp4', '1.mp4')
    time.sleep(1)
    task.run()


