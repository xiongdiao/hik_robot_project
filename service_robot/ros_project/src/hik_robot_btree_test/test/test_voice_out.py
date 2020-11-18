#!/usr/bin/env python

import rospy
import sys
import time
from std_msgs.msg import String, Header
from move_base_msgs.msg import MoveBaseGoal
from geometry_msgs.msg import PoseStamped
from rosjava_hikrobot_msgs.msg import HikRobotVoiceOut

def talker():
    if  len(sys.argv) != 5:
        print "need cmd [group num person cmd]"
        return 0

    voice_pub = rospy.Publisher('HikRobotVoiceOut', HikRobotVoiceOut, queue_size=10)
    rospy.init_node('VoiceOutTest', anonymous=True)
    #voice_pub = rospy.Publisher('VoiceChatter', String, queue_size=10)
    time.sleep(1)

    timeout_t = time.time() + 5
    #print time.time(), timeout_t
    while voice_pub.get_num_connections() == 0 and timeout_t > time.time():
        time.sleep(0.01)

    group = int(sys.argv[1])
    num   = int(sys.argv[2])
    person= int(sys.argv[3])
    cmd   = int(sys.argv[4])

    print group, num, person, cmd
    voice_pub.publish(HikRobotVoiceOut(group, num, person, cmd))
    #print "hello world"
    #voice_pub.publish(String("hello world"))

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
