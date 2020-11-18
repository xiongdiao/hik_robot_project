#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String, Header
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from rosjava_hikrobot_msgs.msg import HikRobotSetTaskMsg

def talker():
    if  len(sys.argv) != 4 :
        print "need cmd [group num cmd]"
        return 0

    pub = rospy.Publisher('HiRobotSetTaskMsg', HikRobotSetTaskMsg, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)
    count = 0
    robot_goal = MoveBaseGoal()

    group = int(sys.argv[1])
    num =   int(sys.argv[2])
    cmd = int(sys.argv[3])

    print group, num, cmd
    pub.publish(HikRobotSetTaskMsg(group, num, cmd,0,0,robot_goal))


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
