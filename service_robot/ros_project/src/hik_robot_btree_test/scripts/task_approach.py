#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import threading
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.pi_trees_lib import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class ApproachTask():
    def __init__(self, name):
        self.need_to_terminate = False
        self.terminate_mutex = threading.RLock()
        self.need_pause = False

        # 初始化行为树
        self.btree_init()
        # 创建task service 服务，根据请求执行对应任务
        #rospy.Service(self.name, HikRobotTaskMngSrv, self.srv_handle)

        # 创建task thread 执行task行为树
        self.execute_thread = threading.Thread(None, self.executeLoop)
        self.execute_thread.start()

    def __del__(self):
        with self.terminate_mutex:
            self.need_to_terminate = True

        assert(self.execute_thread)
        self.execute_thread.join()

    def stop(self):
        print self.name, "stop"
        self.need_pause = True

    def start(self):
        print self.name, "start"
        self.need_pause = False

    def srv_handle(self):
        pass

    def btree_init(self):
        self.root_btree = None


    def executeLoop():
        while (not rospy.is_shutdown()):
            if (self.need_to_terminate):
                break
            #执行行为树
            self.root_btree.run()
            #行为树帧间隔
            time.sleep(0.1)
