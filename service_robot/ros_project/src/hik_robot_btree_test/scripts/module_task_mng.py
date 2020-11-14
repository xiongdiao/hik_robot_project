#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import threading
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.pi_trees_lib import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from task_patrol import PatrolTask
from task_approach import ApproachTask

class TaskMng():
    # 定义一个表，根据group num 直接找到对应处理函数然后做对应动作
    def __init__(self, name):
        self.name = name
        self.terminate_mutex = threading.RLock()
        self.task_list = list() 
        self.current_task = None

        # 初始化 task启动任务管理服务
        self.task_srv = rospy.Service('HikRobotTaskMngSrv', HikRobotTaskMngSrv, self.task_mng_handle)
        # 初始化 task人物结束管理服务
        self.status_srv = rospy.Service('HikRobotStatusSrv', HikRobotStatusSrv, self.status_srv_handle)

        self.execute_thread = threading.Thread(None, self.executeLoop)
        self.execute_thread.start()
        print self.name, "init finish"
    
    def __del__(self):
        assert(self.execute_thread)
        self.execute_thread.join()

    # 接受语音服务转发的task配置请求 启动，暂停，抢占等task任务处理
    def task_mng_handle(self, req):
        # 检查当前执行人物与新任务，并判断执行是否需要抢占，先实现简单直接抢占，后续补充根据紧急情况判断是否能够抢占
        # print "task_mng_handle"
        if self.current_task:
            print "stop current_task", self.current_task
            self.current_task.stop()
            self.task_list.append(self.current_task)

        if req.group == 2 and req.num == 2 and req.cmd == 1 and req.param == 1:
            # 实例化执行全屋巡检task
            print "taskmng start patroltask"
            self.current_task = PatrolTask("PatrolTask")
        elif req.grep == 2 and req.num == 1:
            # 实例化执行靠近老人task
            print "taskmng start approachtask"
            self.current_task = ApproachTask("ApproachTask")

        # 返回1, 确认已收到
        return 1

    # task执行完成后做请求，查询人物list看是否有待执行人物
    def status_srv_handle(self, req):
        #print "status_srv_handle del current task"
        del self.current_task
        self.current_task = None
        if len(self.task_list) != 0:
            #当前任务列表不为空
            print "get task from list"
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
            #print self.name, "loop running"
            
            # 执行频率间隔
            time.sleep(1)
