#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import time
import threading
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from hik_robot_btree_test.msg import *
from hik_robot_btree_test.srv import *
from hik_robot_task.msg import *
from hik_robot_task.srv import *

# 语音服务类，处理语音输入输出指令，分发给不同的模块
VOICEIN_STATUS = 0
VOICEOUT_STATUS = 1

class VoiceService():
    ac_feedback = VoiceOutAcFeedback()
    ac_result = VoiceOutAcResult()
    srv_req = HikRobotSetModulesSrvRequest()

    def __init__(self, name):
        self.name = name

        # 初始化 类状态为等待语音输入请求状态，如果有调用输出交互请求则将状态配置为输出交互状态
        self.status = VOICEIN_STATUS

        # 初始化 安卓模块->语音服务模块 语音输入请求 set task
        rospy.Service("HikRobotSetTaskSrv", HikRobotSetTaskSrv, self.voicein_srv_handle)
        rospy.Service('HikRobotPatrolSrv', HikRobotPatrolSrv, self.handle_patrol_req)
        rospy.Service('HikRobotApproachSrv', HikRobotApproachSrv, self.handle_approach_req)
        rospy.Service('HikRobotSetManPoseSrv', HikRobotSetManPoseSrv, self.handle_setpose_req)

        # 初始化 任务task模块->语音服务模块 语音输出交互请求action server
        self.voiceout_as = actionlib.SimpleActionServer("VoiceOutAcAction", VoiceOutAcAction, execute_cb=self.voiceout_execute_cb, auto_start = False)
        self.voiceout_as.start()

        #初始化 语音服务模块->任务管理模块 task配置服务client句柄
        rospy.wait_for_service('HikRobotTaskMngSrv')
        self.task_handle = rospy.ServiceProxy('HikRobotTaskMngSrv', HikRobotTaskMngSrv)

        # 初始化 语音服务模块->安卓模块 请求安卓发声
        rospy.wait_for_service('HikRobotVoiceOutSrv')
        self.voiceout_handle = rospy.ServiceProxy('HikRobotVoiceOutSrv', HikRobotVoiceOutSrv)

        rospy.loginfo(self.name + " init finish")

    def voicein_srv_handle(self, req):
        if self.status == VOICEIN_STATUS:
            # 执行主动请求的set task指令
            self.set_task(req)
        elif self.status == VOICEOUT_STATUS:
            # 执行被动应答的set task指令
            self.voice_rsp(req)
        else:
            rospy.loginfo("unknow status")

        # 返回1 说明收到消息
        return 1

    def handle_patrol_req(self,  req):
        TaskReq = HikRobotTaskMngSrvRequest()
        TaskReq.group = 2
        TaskReq.num = 2
        TaskReq.cmd = req.cmd
        if self.status == VOICEIN_STATUS:
            # 执行主动请求的set task指令
            self.set_task(TaskReq)
        elif self.status == VOICEOUT_STATUS:
            # 执行被动应答的set task指令
            self.voice_rsp(TaskReq)
        else:
            rospy.loginfo("unknow status")
        return 1

    def handle_approach_req(self, req):
        TaskReq = HikRobotTaskMngSrvRequest()
        TaskReq.group = 2
        TaskReq.num = 1
        TaskReq.cmd = req.cmd
        TaskReq.angle = req.angle
        if self.status == VOICEIN_STATUS:
            rospy.loginfo("handle_approach_req voice in: %d %d", req.cmd, req.angle)
            self.set_task(TaskReq)
        elif self.status == VOICEOUT_STATUS:
            rospy.loginfo("handle_approach_req voice resp: %d %d", req.cmd, req.angle)
            self.voice_rsp(TaskReq)
        else:
            rospy.loginfo("unknow status")

        return 1

    def handle_setpose_req(self, req):
        TaskReq = HikRobotTaskMngSrvRequest()
        TaskReq.group = 2
        TaskReq.num = req.room
        if self.status == VOICEIN_STATUS:
            rospy.loginfo("handle_setpose_req voice in: %d %d", req.group, req.room)
        elif self.status == VOICEOUT_STATUS:
            rospy.loginfo("handle_setpose_req voice resp: %d %d", req.group, req.room)
            self.voice_rsp(TaskReq)
        else:
            rospy.loginfo("unknow status")

        return 1

    def set_task(self, req):
        TaskReq = HikRobotTaskMngSrvRequest()
        TaskReq.group = req.group
        TaskReq.num = req.num
        TaskReq.cmd = req.cmd
        TaskReq.param = req.param
        TaskReq.angle = req.angle
        self.task_handle(TaskReq)

    # 执行语音回应指令->任务模块
    def voice_rsp(self, req):
        self.ac_result.group = req.group
        self.ac_result.num = req.num
        self.ac_result.cmd = req.cmd
        self.status = VOICEIN_STATUS

    # 执行语音输出请求指令回调，进入语音请求状态 语音服务->安卓模块
    def voiceout_execute_cb(self, goal):
        rospy.loginfo(self.name + " voiceout_execute_cb")
        self.status = VOICEOUT_STATUS
        req = HikRobotVoiceOutSrvRequest()
        req.group = goal.group
        req.voicenum = goal.num
        rsp = self.voiceout_handle(req)
        # 等待回复
        while self.status == VOICEOUT_STATUS and not self.voiceout_as.is_preempt_requested(): 
            time.sleep(0.2)
            
        # 只能在execute_cb中执行set_successed
        rospy.loginfo(self.name + " voiceout handle set successed")
        self.voiceout_as.set_succeeded(self.ac_result)
