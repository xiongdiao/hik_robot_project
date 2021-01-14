#!/usr/bin/env python 
# -*- coding: UTF-8 -*-


import rospy
import sys
import signal
import time
import subprocess
from std_msgs.msg import String, Header

from hik_robot_task.srv import *


class PubVideoTask():
    def __init__(self, name, *args, **kwargs):
        self.name = name
        self.req = HikRobotPatrolSrvRequest()
        self.req.cmd = 1
        self.req.num = 2
        rospy.wait_for_service('HikRobotPatrolSrv')
        self.video_client_handle = rospy.ServiceProxy('HikRobotPatrolSrv', HikRobotPatrolSrv)
        print self.name, "init"

    def run(self):
        self.rsp = self.video_client_handle(self.req)
        print self.rsp


if __name__ == '__main__':
    rospy.init_node('srv_test', anonymous=True)
    task = PubVideoTask("srv_test")
    time.sleep(1)
    task.run()



