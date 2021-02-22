#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan

from obstacle_detector.msg import Obstacles

import tf

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs


class enemy_detect():
    def __init__(self):
        self.pose_x = 100
        self.pose_y = 100
        self.next_point = [0.0,0.0,0.0]
        self.near_enemy_range = 0.8
        self.near_enemy = False
        self.near_wall = False
        self.enemy_pub = tf.TransformBroadcaster()

    def judge_enemy(self, ob_x, ob_y, bot_pose_x, bot_pose_y):
        # フィールド内判定のための座標変換
        trans_ob_x = (ob_x*math.cos(math.radians(-45))-ob_y*math.sin(math.radians(-45)))
        trans_ob_y = (ob_x*math.sin(math.radians(-45))+ob_y*math.cos(math.radians(-45)))
        # フイールド内判定
        if (trans_ob_x < -1.15 or trans_ob_x > 1.15) or (trans_ob_y < -1.15 or trans_ob_y >1.15):
            return 0
        # フィールド内のオブジェクトと比較
        if (ob_x > -0.3 and ob_x < 0.3) and (ob_y > -0.3 and ob_y < 0.3):
            return 0
        elif (ob_x < 0.55 and ob_x > 0.47) and (ob_y > 0.4 and ob_y < 0.6):
            return 0
        elif (ob_x > -0.55 and ob_x < -0.47) and (ob_y > 0.4 and ob_y < 0.6):
            return 0
        elif (ob_x > 0.47 and ob_x < 0.55) and (ob_y > -0.6 and ob_y < -0.4):
            return 0
        elif (ob_x > -0.55 and ob_x < -0.47) and (ob_y > -0.6 and ob_y < -0.4):
            return 0
        
        self.pose_x = ob_x
        self.pose_y = ob_y
        self.enemy_pub.sendTransform((ob_x,ob_y,0),
                                tf.transformations.quaternion_from_euler(0, 0, 0),
                                rospy.Time.now(), "d_enemy", "map")

        self.judge_near_enemy(bot_pose_x, bot_pose_y)
        return 0

    def judge_near_enemy(self, bot_pose_x, bot_pose_y):
        x_difference = self.pose_x - bot_pose_x
        y_difference = self.pose_y - bot_pose_y
        #print (math.sqrt((x_difference**2)+(x_difference**2)))
        if math.sqrt((x_difference**2)+(x_difference**2)) <= self.near_enemy_range:
            #print "near_enemy"
            self.near_enemy = True
            
        else:
            self.near_enemy = False
           


class domaeBot():
    def __init__(self):
        self.pose_x = 0
        self.pose_y = 0
        self.th = 0
        self.field_pos = 0
        self.waypoint_number = 0
        self.before_mode_name = "patrol"
        self.near_enemy = False
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)
        self.obstacle_sub = rospy.Subscriber('/tracked_obstacles', Obstacles, self.obstacle_callback)
        self.lider_sub = rospy.Subscriber('/scan', LaserScan, self.lider_callback)
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.e_detect = enemy_detect()

    def pose_callback(self, data):
        self.pose_x = data.pose.pose.position.x
        self.pose_y = data.pose.pose.position.y
        quaternion = data.pose.pose.orientation
        rpy = tf.transformations.euler_from_quaternion((quaternion.x, quaternion.y, quaternion.z, quaternion.w))
        self.th = rpy[2]
        tran_x=(self.pose_x*math.cos(math.radians(45))-self.pose_y*math.sin(math.radians(45)))
        tran_y=(self.pose_x*math.sin(math.radians(45))+self.pose_y*math.cos(math.radians(45)))
      
        if tran_x < 0 and tran_y < 0:
            self.field_pos = 0
        elif tran_x >= 0 and tran_y < 0:
            self.field_pos = 1
        elif tran_x >= 0 and tran_y >= 0:
            self.field_pos = 2
        elif tran_x < 0 and tran_y >= 0:
            self.field_pos = 3

    def obstacle_callback(self, data):
        for i in range(len(data.circles)):
            ob_x = data.circles[i].center.x
            ob_y = data.circles[i].center.y
            self.e_detect.judge_enemy(ob_x, ob_y, self.pose_x, self.pose_y)
            if self.e_detect.near_enemy == True:
                if self.before_mode_name == "patrol":
                    before_waypoint_number = self.waypoint_number
                    self.client.cancel_all_goals()
                    self.waypoint_number = before_waypoint_number
                    self.before_mode_name = "see_enemy"

    def lider_callback(self, data):
        self.scan = data.ranges
        self.near_wall = self.judge_near_wall(self.scan)
            
    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result() 

    def judge_near_wall(self, scan):  
        if not len(scan) == 360:
            return False
        back_scan = scan[:190] + scan[170:]
        back_scan = [x for x in back_scan if x > 0.1]
        if min(back_scan) < 0.2:
            print("back_near_wall")
            return True
        return False
        
        

    # 敵の方向を向いて逃げる
    def see_enemy(self):
        #print("see_enemy")
        th = (math.atan2(self.e_detect.pose_y - self.pose_y, self.e_detect.pose_x - self.pose_x)) - self.th 
        twist = Twist()
        twist.linear.x = -0.05; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        if self.near_wall == True:
            twist.linear.x = 0.01

        #self.client.cancel_goal()
        #print("th = ",th)
        self.vel_pub.publish(twist)
        self.e_detect.near_enemy=False

        return twist
    '''
    def get_next_waypoint(self):
        print("load_waypoint")
        print("field_pos = ",self.field_pos)
        print("mae_waypoint = ",self.waypoint_number)
        if self.field_pos == 0:
            if self.waypoint_number == 1:
                self.waypoint_number = 2
                return [0.0,-0.5,0]
            else: 
                self.waypoint_number = 1
                return [-0.5,0.0,0]
        elif self.field_pos == 1:
            if self.waypoint_number == 3:
                self.waypoint_number = 4
                return [0.5,0.0,3.1415]
            else:
                self.waypoint_number = 3
                return [0.0,-0.5,0]
        elif self.field_pos == 2:
                return [0.5,0.0,3.1415]
        elif self.field_pos == 3:
                return [-0.85,0.45,3.1415/4]
    '''

    def patrol(self):
        if self.e_detect.near_enemy == False:
            self.before_mode_name = "patrol"
            if self.field_pos == 0:
                if not (self.waypoint_number == 1 or self.waypoint_number == 2):  
                    self.setGoal(-0.85,-0.45,-3.1415/4)
                    self.setGoal(-0.85,-0.45,3.1415/2)
                    self.waypoint_number = 1
                elif self.waypoint_number == 1:
                    self.setGoal(-0.85,0.45,3.1415/2)
                    self.setGoal(-0.85,0.45,3.1415/4)
                    self.setGoal(-0.85,0.45,0)
                    self.waypoint_number = 2
                elif self.waypoint_number == 2 or self.waypoint_number == 3:
                    self.setGoal(-0.5,0.0,0)
                    self.setGoal(0.0,-0.5,0)
                    self.waypoint_number = 3
            elif self.field_pos == 1:
                if not (self.waypoint_number == 4):
                    self.setGoal(0.0,-0.5,0)
                    self.setGoal(0.0,-0.5,3.1415*0.5)
                    self.setGoal(0.0,-0.5,3.1415)
                    self.setGoal(0.0,-0.5,0)
                    self.waypoint_number = 4
                elif self.waypoint_number == 4 or self.waypoint_number == 5:
                    self.setGoal(0.5,0.0,3.1415)
                    self.waypoint_number = 5
            elif self.field_pos == 2:
                if not (self.waypoint_number == 6 or self.waypoint_number == 7):
                    self.setGoal(0.5,0.0,3.1415)
                    self.waypoint_number = 6
                elif self.waypoint_number == 6:
                    self.setGoal(0.85,-0.4,0)
                    self.setGoal(0.85,-0.4,-3.1415*(1/4))
                    self.setGoal(0.85,-0.4,-3.1415)
                    self.waypoint_number = 7
                elif self.waypoint_number == 7 or self.waypoint_number == 8:
                    self.setGoal(0.85,0.45,3.1415)
                    self.setGoal(0.0,0.5,3.1415*0.75)
                    self.waypoint_number == 8
            elif self.field_pos == 3:
                if not (self.waypoint_number == 9):
                    self.setGoal(0.0,0.5,3.1415*0.75)
                    self.setGoal(0.0,0.5,3.1415)
                    self.setGoal(0.0,0.5,3.1415*1.5)
                    self.setGoal(0.0,0.5,0)
                    self.waypoint_number = 9
                elif self.waypoint_number == 9:
                    self.setGoal(-0.5,0.0,0)

        elif self.e_detect.near_enemy == True:
            self.see_enemy()

            
    
    def near_enemy(self):
        x_difference = self.pose_x - self.e_detect.pose_x
        y_difference = self.pose_y - self.e_detect.pose_y
        print (math.sqrt((x_difference**2)+(x_difference**2)))
        if math.sqrt((x_difference**2)+(x_difference**2)) <= self.near_enemy_range:
            return True
        else:
            return False
    
    def strategy(self):
        #print ("strate")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            #print("near_enemy",self.e_detect.near_enemy)
            if self.e_detect.near_enemy == False:
                self.patrol()
                #self.next_point = self.get_next_waypoint()
                #self.setGoal(self.next_point[0],self.next_point[1],self.next_point[2])
            elif self.e_detect.near_enemy == True:
                self.see_enemy()
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('domaeBot')
    node = domaeBot()
    node.strategy()