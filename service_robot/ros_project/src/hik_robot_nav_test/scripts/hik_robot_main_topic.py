#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import move_base
import actionlib
from std_msgs.msg import String 
from hik_robot_nav_test.msg import HikRobotSetModulesMsg
from rosjava_hikrobot_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from smach_ros import SimpleActionState  


def patrol_task(msg):
    print "patrol task running"
    my_move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    msg.goal.target_pose.header.frame_id = 'map'
    msg.goal.target_pose.header.stamp = rospy.Time.now()
    my_move_base.send_goal(msg.goal)
    my_move_base.wait_for_result(rospy.Duration(10))

def follow_task(msg):
    if msg.cmd == 0:
        print "follow task stop"
        pub = rospy.Publisher('HiRobotSetModules', HikRobotSetModulesMsg, queue_size=10)
        pub.publish(HikRobotSetModulesMsg(0,0,0))
    else:
        print "follow task start"
        pub = rospy.Publisher('HiRobotSetModules', HikRobotSetModulesMsg, queue_size=10)
        pub.publish(HikRobotSetModulesMsg(0,0,1))

def approching(msg):
    if msg.cmd == 0:
        print "approching task stop"
    elif msg.cmd == 1:
        print "approching task start"

def setPose(msg):
    print "set pose", msg.num
    if msg.num == 0:
        # 到主卧
        pass
    elif msg.num == 1:
        pass
    elif msg.num == 2:
        pass
    elif msg.num == 3:
        pass
    elif msg.num == 4:
        pass
    else:
        pass

def set_task_callback(msg):
    group = msg.group

    position = msg.goal.target_pose.pose.position
    orientation = msg.goal.target_pose.pose.orientation

    print rospy.get_caller_id(), msg.group, msg.num, msg.cmd

    if msg.group == 2:
        if msg.num == 3: #[2, 3] patrol
            patrol_task(msg)
        elif msg.num == 0: #[2, 0] following
            follow_task(msg)
        elif msg.num == 1: #[2, 1] approching
            approching(msg)
        else:
            print "inviled msg"
    elif msg.group == 3:
        setPose(msg)

def listener():
    rospy.init_node('hik_robot_test', anonymous=True)
    rospy.Subscriber("HiRobotSetTaskMsg", HikRobotSetTaskMsg, set_task_callback)
    print('hik robot task Subscriber is ready to get req.')
    rospy.spin()

if __name__ == '__main__':
    listener()
