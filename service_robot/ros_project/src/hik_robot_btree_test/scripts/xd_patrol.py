#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import sys
import signal
import rospy

from module_task_mng import TaskMng
from module_voice_service import VoiceService

def quit(signum, frame):
    print 'stop task'
    rospy.signal_shutdown("quit")
    #sys.exit()

if __name__ == '__main__':
    rospy.init_node('FollowTask')
    signal.signal(signal.SIGINT, quit)
    signal.signal(signal.SIGTERM, quit)

    # 实例化各模块类，然后等待回调执行
    task_mng = TaskMng("task_mng")
    voice_service = VoiceService("voice_out_service")

    rospy.spin()

    del voice_service 
    del task_mng

    print "exit successed"


