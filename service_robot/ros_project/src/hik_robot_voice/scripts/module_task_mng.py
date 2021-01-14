#!/usr/bin/env python
# -*- coding: UTF-8 -*-

'''

task管理模块
任务task管理，优先级处理，暂停，重启等操作

'''

import rospy
import threading
import time

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from hik_robot_btree_test.msg import *
from hik_robot_btree_test.srv import *
from hik_robot_task.msg import *
from hik_robot_task.srv import *
from task_patrol import PatrolTask
from task_approach import ApproachTask

goal_point=[   
    ['主卧', (-4.04510669055, -2.7710217169, -2.93690315345e-07 ), ( -2.50929001864e-06, -1.04532299837e-08, 0.999954645332, 0.0095240365914)],
    ['次卧', (4.28719623994, -1.68696903075, 6.7047681851e-09),    (1.39978919884e-08, 1.58919747342e-08, -0.557240571657, 0.830351097608)],
    ['阳台', (0.112158038761, 5.17669966938, 4.69199491482e-08),   (-1.5094644549e-08, 1.16399754546e-07, 0.734774907242, 0.678311016929)],
    ['客厅', (0, 0, 0),    					   (0, 0, 0, 0)],
    ['书房', (-4.01049644155, 2.2449960999, -2.97643652442e-06),   (-6.6380401294e-06, -3.49535828781e-06, 0.899343173441, 0.437243474884)],  
]

class TaskMng():
    # 定义一个表，根据group num 直接找到对应处理函数然后做对应动作
    def __init__(self, name):
        self.name = name
        self.terminate_mutex = threading.RLock()
        self.task_list = list() 
        self.current_task = None

        # 初始化 task启动任务管理服务
        self.task_srv = rospy.Service('HikRobotTaskMngSrv', HikRobotTaskMngSrv, self.task_mng_handle)
        # 初始化 task任务结束管理服务
        self.status_srv = rospy.Service('HikRobotStatusSrv', HikRobotStatusSrv, self.status_srv_handle)

        self.execute_thread = threading.Thread(None, self.executeLoop)
        self.execute_thread.start()
        rospy.loginfo(self.name + " init finish")
    
    def __del__(self):
        assert(self.execute_thread)
        self.execute_thread.join()

    def handle_current_task(self):
        # 检查当前执行人物与新任务，直接取消旧任务执行新任务
        if self.current_task:
            rospy.loginfo("stop current_task " + self.current_task.name)
            self.current_task.stop()
            del self.current_task
            self.current_task = None
            #self.task_list.append(self.current_task)
    
    def get_task_id(self):
        pass

    # 接受语音服务转发的task配置请求 启动，暂停，抢占等task任务处理
    def task_mng_handle(self, req):
        # 启动任务
        if req.cmd == 1:
            self.handle_current_task()
            rospy.loginfo("taskmng start: %d %d %d", req.group, req.num, req.angle)
            if req.group == 2 and req.num == 2: 
                # 实例化执行全屋巡检task
                self.current_task = PatrolTask("PatrolTask", goal_point)
            elif req.group == 2 and req.num == 1:
                # 实例化执行靠近老人task
                self.current_task = ApproachTask("ApproachTask", req.angle)

        # 停止任务
        elif req.cmd == 0:
            rospy.loginfo("taskmng stop: " + str(req.group) + " " + str(req.num))
            # 查找指令对应任务，并彻底结束任务
            if self.current_task:
                self.current_task.stop()
                del self.current_task
                self.current_task = None

        # 暂停任务
        elif req.cmd == 2:
            rospy.loginfo("taskmng pause: " + str(req.group) + " " + str(req.num))
            # 查找指令对应任务，并暂停任务，暂停功能暂不实现

        # 返回1, 确认已收到
        return 1

    # task执行完成后做请求，查询人物list看是否有待执行人物
    def status_srv_handle(self, req):
        # 检查完成的task是当前task 还是list中的task，并做对应处理
        # to be continue
        rospy.loginfo(self.name + " handle task status req")
        del self.current_task

        self.current_task = None
        if len(self.task_list) != 0:
            #当前任务列表不为空
            rospy.loginfo("get task from list")
            self.current_task = self.task_list.pop()
            self.current_task.start()

        return True

    # task调度管理线程 
    def executeLoop(self):
        while (not rospy.is_shutdown()):
            #with self.terminate_mutex:
            #    if (self.need_to_terminate):
            #        break
        
            # 执行任务管理
            
            # 执行频率间隔
            time.sleep(1)
