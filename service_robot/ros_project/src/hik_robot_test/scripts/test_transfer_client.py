#!/usr/bin/env python 
# -*- coding: UTF-8 -*-


import rospy
import sys
import signal
import time
import subprocess
from std_msgs.msg import String, Header
from hik_robot_test.srv import *


class PubVideoTask():
    def __init__(self, name, path, video_name, *args, **kwargs):
        self.name = name
        self.status = None
        self.video_path = path
        self.video_file = HikRobotFileSrvRequest()
        self.video_file.name = video_name 
        rospy.wait_for_service('HiRobotVideoFile')
        self.video_client_handle = rospy.ServiceProxy('HiRobotVideoFile', HikRobotFileSrv)
        print self.name, "init"

    def run(self):

        print self.name, "run"
        with open("./1.mp4", 'rb') as f:
            data = f.read()
            self.video_file.data = data
            self.rsp = self.video_client_handle(self.video_file)
            print self.rsp

        return self.rsp

if __name__ == '__main__':
    rospy.init_node('VideoFilePub', anonymous=True)
    task = PubVideoTask("PubVideoTask", './', 'recored.avi')
    time.sleep(1)
    task.run()


