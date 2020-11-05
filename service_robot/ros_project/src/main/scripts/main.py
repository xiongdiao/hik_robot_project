#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import sys
import signal

from RobotHw import *
from RobotTaskMng import *
from RobotBtTask import *
from RobotDamon import *

def quit(signum, frame):
    print 'stop task'
    sys.exit()

if __name__ == '__main__':
    rospy.init_node('HikRobotMain')
    signal.signal(signal.SIGINT, quit)
    signal.signal(signal.SIGTERM, quit)
    
    try:
        #HwInit()
        TaskMngInit()
        BtTaskInit()
        #DamonInit()
        pass
    except NameError as err:
        print "sys init error:", err

    print "sys init finish && start working"

    rospy.spin()

