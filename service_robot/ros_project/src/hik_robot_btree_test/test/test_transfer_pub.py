#!/usr/bin/env python 
# -*- coding: UTF-8 -*-


import rospy
import sys
import signal
import time
import subprocess
from std_msgs.msg import String, Header
from hik_robot_test.msg import HikRobotFile
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.pi_trees_lib import *


class PubVideoTask(Task):
    def __init__(self, name, path, video_name, *args, **kwargs):
        super(PubVideoTask, self).__init__(name, *args, **kwargs)
        self.name = name
        self.status = None
        self.video_path = path
        self.video_file = HikRobotFile()
        self.video_file.name = video_name 
        self.video_pub = rospy.Publisher('HiRobotVideoFile', HikRobotFile, queue_size=10)
        print self.name, "init"

    def run(self):

        cmd = ["rosrun", "image_view", "video_recorder", "image:=/camera/rgb/image_raw", "_filename:=" + self.video_path + self.video_file.name]
        p = subprocess.Popen(cmd, stdout=subprocess.PIPE, preexec_fn=os.setsid)
        time.sleep(10)
        try:
            os.killpg(os.getpgid(p.pid), signal.SIGINT)
        except OSError:
            pass
        p.wait()

        self.status = TaskStatus.FAILURE
        print self.name, "run"
        with open(self.video_path + self.video_file.name, 'rb') as f:
            data = f.read()
            self.video_file.data = data
            self.video_pub.publish(self.video_file)
            self.status = TaskStatus.SUCCESS

        return self.status

if __name__ == '__main__':
    rospy.init_node('VideoFilePub', anonymous=True)
    task = PubVideoTask("PubVideoTask", './', 'recored.avi')
    time.sleep(1)
    task.run()


