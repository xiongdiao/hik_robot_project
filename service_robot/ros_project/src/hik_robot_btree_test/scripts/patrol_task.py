#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import threading
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.pi_trees_lib import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

goal_point=[   
    ['主卧', (-4.04510669055, -2.7710217169, -2.93690315345e-07 ), ( -2.50929001864e-06, -1.04532299837e-08, 0.999954645332, 0.0095240365914)],
    ['次卧', (4.28719623994, -1.68696903075, 6.7047681851e-09),    (1.39978919884e-08, 1.58919747342e-08, -0.557240571657, 0.830351097608)],
    ['阳台', (0.112158038761, 5.17669966938, 4.69199491482e-08),   (-1.5094644549e-08, 1.16399754546e-07, 0.734774907242, 0.678311016929)],
    ['客厅', (0, 0, 0),    					   (0, 0, 0, 0)],
    ['书房', (-4.01049644155, 2.2449960999, -2.97643652442e-06),   (-6.6380401294e-06, -3.49535828781e-06, 0.899343173441, 0.437243474884)],  
]

class PatrolTask():
    def __init__(self, name):
        self.name = name
        self.count = 0
        self.need_pause = False
        self.voice_done = False
        self.need_to_terminate = False
        self.terminate_mutex = threading.RLock()

        # 初始化行为树
        self.btree_init()

        # 创建task thread 执行task行为树
        self.execute_thread = threading.Thread(None, self.executeLoop)
        self.execute_thread.start()
        
        # 创建HikRobotStatusSrv client 
        self.set_status = rospy.ServiceProxy('HikRobotStatusSrv', HikRobotStatusSrv)

        # 创建SimpleActionClient client 
        self.voice_ac_client = actionlib.SimpleActionClient('VoiceOutAction', VoiceOutAction)

        # 创建task service 服务，根据请求执行对应任务
        #rospy.Service(self.name, HikRobotTaskSrv, self.srv_handle)

        print name, "init finish"

    def __del__(self):
        print self.name, "del"
        with self.terminate_mutex:
            self.need_to_terminate = True

        #assert(self.execute_thread)
        #self.execute_thread.join()

    def stop(self):
        print self.name, "stop"
        #self.root_btree.stop()
        self.need_pause = True

    def start(self):
        print self.name, "start"
        #self.root_btree.start()
        self.need_pause = False

    # 初始化人物行为树，添加各节点
    def btree_init(self):
        self.root_btree = None

    def srv_handle(self):
        pass

    def voice_done_cb(self, state, result):
        self.voice_done = True
        print "voice_done_cb running"

    def set_task_status(self, status):
        StatusReq = HikRobotStatusSrvRequest()
        StatusReq.group = 2 
        StatusReq.num = 2 
        StatusReq.cmd = 1 
        StatusReq.param = status 
        self.set_status(StatusReq)

    def executeLoop(self):
        while (not rospy.is_shutdown()):
            if (self.need_to_terminate):
                break

            if self.need_pause:
                #self.root_btree.stop()
                time.sleep(0.1)
                continue
        
            #执行行为树
            #status = self.root_btree.run()
            status = TaskStatus.RUNNING
            if self.count >= 5 and self.voice_done == True:
                status = TaskStatus.SUCCESS

            if status == TaskStatus.SUCCESS:
                # 执行success后，上报任务状态已完成
                with self.terminate_mutex:
                    self.need_to_terminate = True
                self.set_task_status(1)
                print self.name, "finish success"

            elif status == TaskStatus.RUNNING:
                self.count = self.count + 1
                if self.count == 2:
                    goal = VoiceOutGoal()
                    goal.group = 1
                    goal.num = 2
                    goal.person = 3
                    goal.cmd = 4
                    # 验证行为树节点中调用action功能
                    self.voice_ac_client.send_goal(goal, done_cb = self.voice_done_cb)
                    print self.name, "send voice goal"

            elif status == TaskStatus.FAILURE:
                # 执行failure后，上报状态任务执行失败
                with self.terminate_mutex:
                    self.need_to_terminate = True
                self.set_task_status(0)
                print self.name, "Running failure"
            else:
                print "unknow status"
            
            #print self.name, "loop"
            #行为树帧间隔
            time.sleep(0.1)

