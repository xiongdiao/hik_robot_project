#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import time
import sys
import signal
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.pi_trees_lib import *
from hik_robot_test.msg import HikRobotSetModulesMsg
from hik_robot_test.srv import HikRobotSetModulesSrv
from rosjava_hikrobot_msgs.msg import HikRobotSetTaskMsg, HikRobotVoiceOut

goal_point=[   
    ['主卧', (-4.04510669055, -2.7710217169, -2.93690315345e-07 ), ( -2.50929001864e-06, -1.04532299837e-08, 0.999954645332, 0.0095240365914)],
    ['次卧', (4.28719623994, -1.68696903075, 6.7047681851e-09),    (1.39978919884e-08, 1.58919747342e-08, -0.557240571657, 0.830351097608)],
    ['阳台', (0.112158038761, 5.17669966938, 4.69199491482e-08),   (-1.5094644549e-08, 1.16399754546e-07, 0.734774907242, 0.678311016929)],
    ['客厅', (0, 0, 0),    					   (0, 0, 0, 0)],
    ['书房', (-4.01049644155, 2.2449960999, -2.97643652442e-06),   (-6.6380401294e-06, -3.49535828781e-06, 0.899343173441, 0.437243474884)],  
]

def quit(signum, frame):
    print 'stop task'
    sys.exit()


class ApproachTask():
    def __init__(self):
        # The root node
        ApproachTaskSeqNode = Sequence("FollowTask", reset_after = True)
        self.node = ApproachTaskSeqNode
        self.goal = None
        self.NeedApproach = False

        rospy.Subscriber("HiRobotSetTaskMsg", HikRobotSetTaskMsg, self.approachTaskMsgCb, tcp_nodelay=True)
        rospy.Service('HikRobotFollowTaskSrv', HikRobotSetModulesSrv, self.approachTaskSrvCb)

        CheckPoseActionNode =  CheckPoseAction("CheckPoseActionNode")
        ApproachActionNode = ApproachAction("ApproachActionNode")

        # Add the nodes to the approach task
        ApproachTaskSeqNode.add_child(CheckPoseActionNode)
        ApproachTaskSeqNode.add_child(ApproachActionNode)

        # Print a simple representation of the tree
        print "Behavior Tree Structure"
        print_tree(ApproachTaskSeqNode, use_symbols=True)

        # Run the tree
        while True:
            if not self.NeedApproach:
                time.sleep(1)
                continue

            status = ApproachTaskSeqNode.run()
            if status == TaskStatus.SUCCESS:
                # 执行success后，上报任务状态已完成
                self.NeedApproach = False
                print "Finished running tree."
            elif status == TaskStatus.RUNNING:
                print "ApproachTaskSeqNode Running"
            elif status == TaskStatus.FAILURE:
                # 执行failure后，上报状态任务执行失败
                self.NeedApproach = False
                print "Running failure"
            else:
                print "unknow status"

            time.sleep(1)

    def approachTaskMsgCb(self, msg):
        print "approachTaskMsgCb"
        if msg.group == 2 and msg.num == 1:
            if msg.cmd == 0:
                self.NeedApproach = False
                self.node.cancel()
                print "cancel task"
            else:
                self.NeedApproach = True

    def approachTaskSrvCb(self, req):
        if req.group == 0 and req.num == 0:
            if req.param == 0:
                self.NeedApproach = False
                self.node.cancel()
                print "cancel task"
            else:
                self.NeedApproach = True
        return True

class ApproachAction(Task):
    def __init__(self, name, *args, **kwargs):
        super(ApproachAction, self).__init__(name, *args, **kwargs)
        self.name = name
        self.goal = MoveBaseGoal()

        rospy.Subscriber("HiRobotSetTaskMsg", HikRobotSetTaskMsg, self.approachTaskMsgCb, tcp_nodelay=True)
        self.voice_pub = rospy.Publisher('HikRobotVoiceOut', HikRobotVoiceOut, queue_size=10)
        # 定义action 执行move_bace_action, 在回调中配置action状态为successed
        self.moveBaseAction = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        print "Creating ApproachAction"

    def run(self):
        print self.name, "running"
        if self.status == None:
            #第一次开始执行，进入RUNNING状态，等待回调配置坐标，然后执行导航到目的位置
            self.status = TaskStatus.RUNNING

        return self.status

    def approachTaskMsgCb(self, msg):
        print self.name, "approachTaskMsgCb"
        if msg.group == 3:
            if msg.num == 0:
                print "到主卧"
                self.goal.target_pose.header.frame_id = 'map'
                self.goal.target_pose.header.stamp = rospy.Time.now()
                self.goal.target_pose.pose.position.x    = goal_point[0][1][0]
                self.goal.target_pose.pose.position.y    = goal_point[0][1][1]
                self.goal.target_pose.pose.position.z    = goal_point[0][1][2]
                self.goal.target_pose.pose.orientation.x = goal_point[0][2][0]
                self.goal.target_pose.pose.orientation.y = goal_point[0][2][1]
                self.goal.target_pose.pose.orientation.z = goal_point[0][2][2]
                self.goal.target_pose.pose.orientation.w = goal_point[0][2][3]
                self.moveBaseAction.send_goal(self.goal, active_cb = self.actionActiveCb, done_cb = self.actionDoneCb)
            elif msg.num == 1:
                print "到次卧"
                self.status = TaskStatus.SUCCESS
            elif msg.num == 2:
                print "到阳台"
                self.status = TaskStatus.SUCCESS
            elif msg.num == 3:
                print "到客厅"
                self.status = TaskStatus.SUCCESS
            elif msg.num == 4:
                print "到厨房"
                self.status = TaskStatus.SUCCESS

    def actionActiveCb(self):
        print self.name, "action active successed"

    def actionDoneCb(self, state, result):
        if state == 3:
            self.voice_pub.publish(HikRobotVoiceOut(4, 0, 0, 0))
            self.status = TaskStatus.SUCCESS

        print "actionDoneCb", state, result


class CheckPoseAction(Task):
    def __init__(self, name, *args, **kwargs):
        super(CheckPoseAction, self).__init__(name, *args, **kwargs)
        self.name = name
        self.status = None

        #rospy.Subscriber("HikRobotCheckPoseMsg", HikRobotCheckPoseMsg, self.CheckPoseMsgCb, tcp_nodelay=True)
        #rospy.Service('HikRobotCheckPoseSrv', HikRobotCheckPoseSrv, self.CheckPoseSrvCb)
        self.voice_pub = rospy.Publisher('HikRobotVoiceOut', HikRobotVoiceOut, queue_size=10)
        print "Creating CheckPoseAction"

    # 如果在无状态情况下，则自动进入running状态，等待消息/服务回调配置进入其他状态
    def run(self):
        if self.status == None:
            print self.name, "req coordinate"
            # 第一次进入，检查老人是否在相机中，在返回成功，不在发送语音询问请求然后返回running状态
            self.voice_pub.publish(HikRobotVoiceOut(1, 0, 0, 0))
            self.status = TaskStatus.SUCCESS
        elif self.status == TaskStatus.SUCCESS:
            print self.name, "success"

        return self.status

    def CheckPoseMsgCb(self, msg):
        print self.name, "check pose msg cb"

    def CheckPoseSrvCb(self, req):
        print self.name, "check pose srv cb"

if __name__ == '__main__':
    rospy.init_node('FollowTask')
    signal.signal(signal.SIGINT, quit)
    signal.signal(signal.SIGTERM, quit)

    tree = ApproachTask()


