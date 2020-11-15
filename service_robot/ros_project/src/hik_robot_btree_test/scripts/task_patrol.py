#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import threading
from pi_trees_ros.pi_trees_ros import *
from pi_trees_lib.pi_trees_lib import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry 

from hik_robot_test.msg import *
from hik_robot_test.srv import *

goal_point=[   
    ['主卧', (-4.04510669055, -2.7710217169, -2.93690315345e-07 ), ( -2.50929001864e-06, -1.04532299837e-08, 0.999954645332, 0.0095240365914)],
    ['次卧', (4.28719623994, -1.68696903075, 6.7047681851e-09),    (1.39978919884e-08, 1.58919747342e-08, -0.557240571657, 0.830351097608)],
    ['阳台', (0.112158038761, 5.17669966938, 4.69199491482e-08),   (-1.5094644549e-08, 1.16399754546e-07, 0.734774907242, 0.678311016929)],
    ['书房', (-4.01049644155, 2.2449960999, -2.97643652442e-06),   (-6.6380401294e-06, -3.49535828781e-06, 0.899343173441, 0.437243474884)],  
]

class PatrolTask():
    def __init__(self, name, goal_list):
        self.name = name
        self.count = 0
        self.need_pause = False
        self.voice_done = False
        self.need_to_terminate = False
        self.terminate_mutex = threading.RLock()
        self.current_goal = MoveBaseGoal()
        self.move_action = []

        print "goal list len:", len(goal_list)
        #rospy.loginfo("goal list len:", len(goal_list))

        
        # 创建HikRobotStatusSrv client 
        #rospy.wait_for_service('HikRobotStatusSrv')
        self.set_status = rospy.ServiceProxy('HikRobotStatusSrv', HikRobotStatusSrv)

        # 创建SimpleActionClient client 
        self.voice_ac_client = actionlib.SimpleActionClient('VoiceOutAction', VoiceOutAction)

        # 创建task service 服务，根据请求执行对应任务
        #rospy.Service(self.name, HikRobotTaskSrv, self.srv_handle)

        # 订阅坐标信息，存储当前坐标位置
        #rospy.Subscriber("odom", Odometry, self.get_Odome)

        # 初始化行为树
        self.btree_init(goal_list)

        # 创建task thread 执行task行为树
        self.execute_thread = threading.Thread(None, self.executeLoop)
        self.execute_thread.start()

        print name, "init finish"

    def __del__(self):
        print self.name, "del"
        with self.terminate_mutex:
            self.need_to_terminate = True

        self.root_btree.reset()
        #assert(self.execute_thread)
        #self.execute_thread.join()

    # 初始化人物行为树，添加各节点
    def btree_init(self, goal_list):
        self.root_btree = Sequence("FollowTask", reset_after = False)
        # 添加巡检点
        for i in range(0, len(goal_point)):
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x    = goal_point[i][1][0]
            goal.target_pose.pose.position.y    = goal_point[i][1][1]
            goal.target_pose.pose.position.z    = goal_point[i][1][2]
            goal.target_pose.pose.orientation.x = goal_point[i][2][0]
            goal.target_pose.pose.orientation.y = goal_point[i][2][1]
            goal.target_pose.pose.orientation.z = goal_point[i][2][2]
            goal.target_pose.pose.orientation.w = goal_point[i][2][3]
            self.move_action.append(SimpleActionTask("approachAction" + str(i), 'move_base', MoveBaseAction, goal, result_timeout = 600))
            self.root_btree.add_child(self.move_action[i])

        # 添加当前位置
        self.get_current_pose()
        self.move_action.append(SimpleActionTask("approachCurrent", 'move_base', MoveBaseAction, self.current_goal, result_timeout = 600))
        self.root_btree.add_child(self.move_action[len(self.move_action) - 1])

        print_tree(self.root_btree, use_symbols=True)

    def stop(self):
        print self.name, "stop"
        #self.root_btree.stop()
        self.need_pause = True

    def start(self):
        print self.name, "start"
        #self.root_btree.start()
        self.need_pause = False

    def get_current_pose(self):
        current_odom = rospy.wait_for_message('/odom', Odometry, timeout=5)
        position    = current_odom.pose.pose.position
        orientation = current_odom.pose.pose.orientation 
        goal        = self.current_goal
        goal.target_pose.header.frame_id    = 'map'
        goal.target_pose.pose.position.x    = position.x
        goal.target_pose.pose.position.y    = position.y
        goal.target_pose.pose.position.z    = position.z
        goal.target_pose.pose.orientation.x = orientation.x
        goal.target_pose.pose.orientation.y = orientation.y
        goal.target_pose.pose.orientation.z = orientation.z
        goal.target_pose.pose.orientation.w = orientation.w

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
            status = self.root_btree.run()

            if status == TaskStatus.SUCCESS:
                # 执行success后，上报任务状态已完成
                with self.terminate_mutex:
                    self.need_to_terminate = True
                self.set_task_status(1)
                print self.name, "finish success"

            elif status == TaskStatus.RUNNING:
                #print self.name, "Running running"
                pass

            elif status == TaskStatus.FAILURE:
                # 执行failure后，上报状态任务执行失败
                with self.terminate_mutex:
                    self.need_to_terminate = True
                self.set_task_status(0)
                print self.name, "Running failure"

            else:
                print "unknow status"
            
            #print self.name, "loop"
            #行为树帧间隔
            time.sleep(0.1)

