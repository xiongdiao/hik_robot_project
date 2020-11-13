#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import time
import sys
import signal
import rospy
import threading
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.pi_trees_lib import *
from hik_robot_test.srv import * 
from hik_robot_test.msg import *

goal_point=[   
    ['主卧', (-4.04510669055, -2.7710217169, -2.93690315345e-07 ), ( -2.50929001864e-06, -1.04532299837e-08, 0.999954645332, 0.0095240365914)],
    ['次卧', (4.28719623994, -1.68696903075, 6.7047681851e-09),    (1.39978919884e-08, 1.58919747342e-08, -0.557240571657, 0.830351097608)],
    ['阳台', (0.112158038761, 5.17669966938, 4.69199491482e-08),   (-1.5094644549e-08, 1.16399754546e-07, 0.734774907242, 0.678311016929)],
    ['客厅', (0, 0, 0),    					   (0, 0, 0, 0)],
    ['书房', (-4.01049644155, 2.2449960999, -2.97643652442e-06),   (-6.6380401294e-06, -3.49535828781e-06, 0.899343173441, 0.437243474884)],  
]


def quit(signum, frame):
    print 'stop task'
    rospy.signal_shutdown("quit")
    #sys.exit()


# 语音服务类，处理语音输入输出指令，分发给不同的模块
VOICEIN_STATUS = 0
VOICEOUT_STATUS = 1
class VoiceService():
    ac_feedback = VoiceOutFeedback()
    ac_result = VoiceOutResult()
    srv_req = HikRobotSetModulesSrvRequest()

    def __init__(self, name):
        self.name = name

        # 初始化 类状态为等待语音输入请求状态，如果有调用输出交互请求则将状态配置为输出交互状态
        self.status = VOICEIN_STATUS

        # 初始化 安卓模块->语音服务模块 语音输入请求 set task
        self.voicein_srv = rospy.Service("HikRobotSetTaskSrv", HikRobotSetTaskSrv, self.voicein_srv_handle)

        # 初始化 任务task模块->语音服务模块 语音输出交互请求action server
        self.voiceout_as = actionlib.SimpleActionServer("VoiceOutAction", VoiceOutAction, execute_cb=self.voiceout_execute_cb, auto_start = False)
        self.voiceout_as.start()

        #初始化 语音服务模块->任务管理模块 task配置服务client句柄
        rospy.wait_for_service('HikRobotTaskMngSrv')
        self.task_handle = rospy.ServiceProxy('HikRobotTaskMngSrv', HikRobotTaskMngSrv)

        # 初始化 语音服务模块->安卓模块 请求安卓发声
        rospy.wait_for_service('HikRobotVoiceOutSrv')
        self.voiceout_handle = rospy.ServiceProxy('HikRobotVoiceOutSrv', HikRobotVoiceOutSrv)

        print self.name, "init finish"

    def voicein_srv_handle(self, req):
        if self.status == VOICEIN_STATUS:
            # 执行主动请求的set task指令
            #print("runing VOICEIN_STATUS")
            self.set_task(req)
        elif self.status == VOICEOUT_STATUS:
            # 执行被动应答的set task指令
            #print("runing VOICEOUT_STATUS")
            self.voice_rsp(req)
        else:
            print("unknow status")

        # 返回1 说明收到消息
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
        print self.name, "voiceout_execute_cb"
        self.status = VOICEOUT_STATUS
        req = HikRobotVoiceOutSrvRequest()
        req.group = goal.group
        rsp = self.voiceout_handle(req)
        print self.name, "voiceout handle", rsp
        # 等待回复
        while self.status == VOICEOUT_STATUS and not self.voiceout_as.is_preempt_requested(): 
            time.sleep(0.2)
            
        # 只能在execute_cb中执行set_successed
        self.voiceout_as.set_succeeded(self.ac_result)

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
        #print "task_mng_handle"
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


# 独立线程执行
class PatrolTask():
    def __init__(self, name):
        #self.req = req
        self.name = name
        self.count = 0
        self.need_pause = False
        self.need_to_terminate = False
        self.terminate_mutex = threading.RLock()
        # 初始化行为树
        self.btree_init()
        self.voice_done = False
        # 创建task thread 执行task行为树
        self.execute_thread = threading.Thread(None, self.executeLoop)
        self.execute_thread.start()
        
        self.set_status = rospy.ServiceProxy('HikRobotStatusSrv', HikRobotStatusSrv)
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
        self.need_pause = True

    def start(self):
        print self.name, "start"
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
                print self.name, "Finished running tree."
                with self.terminate_mutex:
                    print "set terminate true."
                    self.need_to_terminate = True
                self.set_task_status(1)

            elif status == TaskStatus.RUNNING:
                self.count = self.count + 1
                if self.count == 2:
                    goal = VoiceOutGoal()
                    goal.group = 1
                    goal.num = 2
                    goal.person = 3
                    goal.cmd = 4
                    self.voice_ac_client.send_goal(goal, done_cb = self.voice_done_cb)
                    print self.name, "send voice goal"

            elif status == TaskStatus.FAILURE:
                # 执行failure后，上报状态任务执行失败
                with self.terminate_mutex:
                    self.need_to_terminate = True

                self.set_status(rsp)
                print self.name, "Running failure"
            else:
                print "unknow status"
            
            print self.name, "loop"
            #行为树帧间隔
            time.sleep(1)

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

    def srv_handle():
        pass

    def btree_init():
        self.root_btree = None

    def executeLoop():
        
        while (not rospy.is_shutdown()):
            if (self.need_to_terminate):
                break
        
            #执行行为树
            self.root_btree.run()
            
            #行为树帧间隔
            time.sleep(0.1)


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


