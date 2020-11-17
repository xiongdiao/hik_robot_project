#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import threading
from nav_msgs.msg import Odometry 
from actionlib_msgs.msg import GoalStatus
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.pi_trees_lib import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

goal_point=[   
    ['主卧', (-4.04510669055, -2.7710217169, -2.93690315345e-07 ), ( -2.50929001864e-06, -1.04532299837e-08, 0.999954645332, 0.0095240365914)],
    ['次卧', (4.28719623994, -1.68696903075, 6.7047681851e-09),    (1.39978919884e-08, 1.58919747342e-08, -0.557240571657, 0.830351097608)],
    ['阳台', (0.112158038761, 5.17669966938, 4.69199491482e-08),   (-1.5094644549e-08, 1.16399754546e-07, 0.734774907242, 0.678311016929)],
    ['书房', (-4.01049644155, 2.2449960999, -2.97643652442e-06),   (-6.6380401294e-06, -3.49535828781e-06, 0.899343173441, 0.437243474884)],  
]

class ApproachTask():
    def __init__(self, name):
        self.name = name
        self.count = 0
        self.need_pause = False
        self.voice_done = False
        self.need_to_terminate = False
        self.terminate_mutex = threading.RLock()

        # 创建HikRobotStatusSrv client 
        #rospy.wait_for_service('HikRobotStatusSrv')
        self.set_status = rospy.ServiceProxy('HikRobotStatusSrv', HikRobotStatusSrv)
        
        # 开启目标识别，目标识别叶节点在确认检测到目标后，当前任务树返回成功
        # 待算法提供topic，叶节点直接订阅消息并

        # 初始化行为树
        self.btree_init(angle=0.5)

        # 创建task thread 执行task行为树
        self.execute_thread = threading.Thread(None, self.executeLoop)
        self.execute_thread.start()

    def __del__(self):
        rospy.loginfo(self.name + " del")
        with self.terminate_mutex:
            self.need_to_terminate = True

        self.root_btree.reset()

    def btree_init(self, angle=0):
        self.root_btree = Selector("ApproachSelTask", reset_after = False)
        ApproachTargetSeqNode = Sequence("ApproachTargetSeqnode", reset_after=False)

        FirstCheckTargetNode = CheckTarget("FirstCheckTargetAction")

        TurnArroundActionNode = TurnArroundAction("TurnArroundActionNode", angle)
        ApproachActionNode = ApproachAction("ApproachActionNode")
        FinalCheckTargetNode = CheckTarget("FinalCheckTargetNode", status=TaskStatus.SUCCESS)
        ApproachTargetSeqNode.add_child(TurnArroundActionNode)
        ApproachTargetSeqNode.add_child(ApproachActionNode)
        ApproachTargetSeqNode.add_child(FinalCheckTargetNode)

        self.root_btree.add_child(FirstCheckTargetNode)
        self.root_btree.add_child(ApproachTargetSeqNode)

        print_tree(self.root_btree, use_symbols=True)

    def pause(self):
        rospy.loginfo(self.name + " pause")
        #self.root_btree.pause()
        self.need_pause = True

    def stop(self):
        rospy.loginfo(self.name + " stop")
        self.need_to_terminate = True
        self.root_btree.reset()

    def start(self):
        rospy.loginfo(self.name + " start")
        self.need_pause = False

    def srv_handle(self):
        pass

    def set_task_status(self, status):
        StatusReq = HikRobotStatusSrvRequest()
        StatusReq.group = 2 
        StatusReq.num = 1 
        StatusReq.id  = 0
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
            status = self.root_btree.run()
            if status == TaskStatus.SUCCESS:
                # 执行success后，上报任务状态已完成
                with self.terminate_mutex:
                    self.need_to_terminate = True
                self.set_task_status(1)
                rospy.loginfo(self.name + " finish success")

            elif status == TaskStatus.RUNNING:
                rospy.loginfo(self.name + " running")
                pass

            elif status == TaskStatus.FAILURE:
                # 执行failure后，上报状态任务执行失败
                with self.terminate_mutex:
                    self.need_to_terminate = True
                self.set_task_status(0)
                rospy.loginfo(self.name + " failure")

            else:
                rospy.loginfo("unknow status")
            

            #行为树帧间隔
            time.sleep(1)

class TurnArroundAction(Task):
    def __init__(self, name, angle, *args, **kwargs):
        super(TurnArroundAction, self).__init__(name, *args, **kwargs)
        self.name = name
        self.status = None
        self.action_finished = False
        self.action_started = False
        rospy.loginfo("Creating TurnArroundAction angle: " + str(angle))
        self.target_goal = MoveBaseGoal()
        current_odom = rospy.wait_for_message('/odom', Odometry, timeout=5)
        position    = current_odom.pose.pose.position
        orientation = current_odom.pose.pose.orientation 
        z = orientation.z + angle
        if z > 1:
            z = z-2
        goal        = self.target_goal
        goal.target_pose.header.frame_id    = 'map'
        goal.target_pose.pose.position    = position
        goal.target_pose.pose.orientation = orientation
        goal.target_pose.pose.orientation.z = z
        self.turn_arround_task = SimpleActionTask("TurnArroundAction", 'move_base', MoveBaseAction, self.target_goal, result_timeout = 600)

    # 如果在无状态情况下，则自动进入running状态，等待消息/服务回调配置进入其他状态
    def run(self):
        if not self.action_started:
            self.action_started = True

        status = self.turn_arround_task.run()
        if status == TaskStatus.SUCCESS:
            rospy.loginfo(self.name + " finish success")
        elif status == TaskStatus.RUNNING:
            rospy.loginfo(self.name + " running")
        elif status == TaskStatus.SUCCESS:
            rospy.loginfo(self.name + " running")
            self.action_finished = True
        elif status == TaskStatus.FAILURE:
            rospy.loginfo(self.name + " failure")
            self.action_finished = True
        else:
            rospy.loginfo("unknow status")

        return status

    def reset(self):
        rospy.loginfo(self.name + " reset")
        if not self.action_finished and self.action_started == True:
            rospy.loginfo(self.name + " turn arround task reset")
            self.turn_arround_task.reset()

        self.action_finished = False
        self.turn_arround_task = None
        self.action_started = False

class ApproachAction(Task):
    def __init__(self, name, *args, **kwargs):
        super(ApproachAction, self).__init__(name, *args, **kwargs)
        self.name = name
        self.status = TaskStatus.RUNNING
        self.action_finished = False
        self.action_started = False

        rospy.loginfo("Creating ApproachAction start")
        # 创建 VoiceOutAction 请求声音交互，并获取交互结果
        self.voice_ac_client = actionlib.SimpleActionClient('VoiceOutAction', VoiceOutAction)
        self.voice_ac_finished = False
        time.sleep(1)
        self.voice_goal = VoiceOutGoal() 
        self.voice_goal.group = 2
        self.voice_goal.num = 1
        self.approach_task = None
        rospy.loginfo("Creating ApproachAction finish")

    def run(self):
        if not self.action_started:
            self.voice_ac_client.send_goal(self.voice_goal,
                                        #active_cb = callback_active,
                                        #feedback_cb = callback_feedback,
                                        done_cb = self.callback_done)
            self.action_started = True

        if not self.action_finished:
            if self.approach_task:
                self.status = self.approach_task.run()
            else:
                rospy.loginfo(self.name + " wait for target pose")

            if self.status == TaskStatus.FAILURE:
                rospy.loginfo(self.name + " failure")
                self.action_finished = True
            elif self.status == TaskStatus.SUCCESS:
                rospy.loginfo(self.name + " success")
                self.action_finished = True
            elif self.status == TaskStatus.RUNNING:
                rospy.loginfo(self.name + " running")
 
        return self.status

    def callback_done(self, state, result):
        rospy.loginfo(self.name + " callback done")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x    = goal_point[0][1][0]
        goal.target_pose.pose.position.y    = goal_point[0][1][1]
        goal.target_pose.pose.position.z    = goal_point[0][1][2]
        goal.target_pose.pose.orientation.x = goal_point[0][2][0]
        goal.target_pose.pose.orientation.y = goal_point[0][2][1]
        goal.target_pose.pose.orientation.z = goal_point[0][2][2]
        goal.target_pose.pose.orientation.w = goal_point[0][2][3]
        self.approach_task = SimpleActionTask("ApproachAction", 'move_base', MoveBaseAction, goal, result_timeout = 600)
        self.voice_ac_finished = True

    def reset(self):
        rospy.loginfo(self.name + " reset")
        if GoalStatus.ACTIVE == self.voice_ac_client.get_state() or GoalStatus.PENDING == self.voice_ac_client.get_state():
            rospy.loginfo(self.name + " cancel goal")
            self.voice_ac_client.cancel_goal()

        if self.approach_task and not self.action_finished:
            self.approach_task.reset()

        self.voice_ac_finished = False
        self.action_finished = False
        self.approach_task = None
        self.action_started = False


class CheckTarget(Task):
    def __init__(self, name, status=TaskStatus.FAILURE, *args, **kwargs):
        super(CheckTarget, self).__init__(name, *args, **kwargs)
        self.name = name
        self.action_started = False
        self.status = status

    # 如果在无状态情况下，则自动进入running状态，等待消息/服务回调配置进入其他状态
    def run(self):
        # 默认检测不到
        rospy.loginfo(self.name + " status " + str(self.status))
        return self.status

    def reset(self):
        rospy.loginfo(self.name + " reset")
        self.action_started = False
