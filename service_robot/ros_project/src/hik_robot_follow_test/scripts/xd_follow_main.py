#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import time
import sys
import signal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.pi_trees_lib import *
from hik_robot_nav_test.msg import HikRobotSetModulesMsg
from hik_robot_nav_test.srv import HikRobotSetModulesSrv

def quit(signum, frame):
    print 'stop task'
    sys.exit()

class FollowTask():
    def __init__(self):
        # The root node
        FollowTaskSeqNode = Sequence("FollowTask", reset_after = True)
        self.node = FollowTaskSeqNode
        # Create follow task sub
        rospy.Subscriber("HikRobotFollowTaskMsg", HikRobotSetModulesMsg, self.followTaskMsgCb, tcp_nodelay=True)

        rospy.Service('HikRobotFollowTaskSrv', HikRobotSetModulesSrv, self.followTaskSrvCb)

        # Create follow action
        FollowActionNode = FollowAction("FollowActionNode", 1, 5, 1)

        #PrepareCaramerSeq = Selector("PrepareCaramerSel")
        #TurnAroundActionNode = TurnAroundAction()

        #CheckCarmerSel = Selector("CheckTargetSel")
        #CheckCarmerConditionNode = CheckCarmerCondition("CheckCarmerCondition")

        # Add the nodes to the follow task
        FollowTaskSeqNode.add_child(FollowActionNode)

        # Print a simple representation of the tree
        print "Behavior Tree Structure"
        print_tree(FollowTaskSeqNode, use_symbols=True)

        self.NeedFollow = False

        # Run the tree
        while True:
            if not self.NeedFollow:
                print "FollowTaskSeqNode dont need follow"
                time.sleep(1)
                continue

            status = FollowTaskSeqNode.run()
            if status == TaskStatus.SUCCESS:
                # 执行success后，上报任务状态已完成
                self.NeedFollow = False
                print "Finished running tree."
            elif status == TaskStatus.RUNNING:
                print "Running status"
            elif status == TaskStatus.FAILURE:
                # 执行failure后，上报状态任务执行失败
                self.NeedFollow = False
                print "Running failure"
            else:
                print "unknow status"
            time.sleep(1)

    def followTaskMsgCb(self, msg):
        print "followTaskMsgCb"
        if msg.group == 0 and msg.num == 0:
            if msg.param == 0:
                self.NeedFollow = False
                self.node.cancel()
                print "cancel task"
            else:
                self.NeedFollow = True

    def followTaskSrvCb(self, req):
        if req.group == 0 and req.num == 0:
            if req.param == 0:
                self.NeedFollow = False
                self.node.cancel()
                print "cancel task"
            else:
                self.NeedFollow = True
        return True

class FollowAction(Task):
    def __init__(self, name, start, stop, step, *args, **kwargs):
        super(FollowAction, self).__init__(name, *args, **kwargs)
        self.name = name
        self.start = start
        self.stop = stop
        self.step = step
        self.count = self.start
        print "Creating FollowAction", self.start, self.stop, self.step

    def run(self):
        print self.name, "running"
        self.count += self.step
        print self.count, self.stop
        if self.count >= self.stop:
            return TaskStatus.SUCCESS
        else:
            return TaskStatus.RUNNING

    def reset(self):
        print self.name, "reset"
        self.count = 0

if __name__ == '__main__':
    rospy.init_node('FollowTask')
    signal.signal(signal.SIGINT, quit)
    signal.signal(signal.SIGTERM, quit)

    tree = FollowTask()


