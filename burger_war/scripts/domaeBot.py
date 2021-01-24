#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import random

from geometry_msgs.msg import Twist

import tf


import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs

# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

#from std_msgs.msg import String
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
#import cv2


class domaeBot():
    def __init__(self):
        
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)




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
    
    def patrol(self):
        #巡回ルート
        #右下
        self.setGoal(-0.85,-0.45,-3.1415/4)
        self.setGoal(-0.85,-0.45,3.1415/2)
        #左下
        self.setGoal(-0.85,0.45,3.1415/2)
        self.setGoal(-0.85,0.45,3.1415/4)
        #中央下
        self.setGoal(-0.5,0.0,0)
        #右
        #self.setGoal(-0.36,-0.25,-3.1415/4)
        self.setGoal(0.0,-0.5,0)
        self.setGoal(0.0,-0.5,3.1415*0.5)
        self.setGoal(0.0,-0.5,3.1415)
        self.setGoal(0.0,-0.5,0)
        ###################################

        #中央下
        self.setGoal(-0.5,0.0,0)
        #右下
        self.setGoal(-0.85,-0.45,-3.1415/4)
        self.setGoal(-0.85,-0.45,3.1415/2)
        #左下
        self.setGoal(-0.85,0.45,3.1415/2)
        self.setGoal(-0.85,0.45,3.1415/4)

        #左
        #self.setGoal(0.36,0.28,3.1415*0.75)
        self.setGoal(0.0,0.5,3.1415*0.75)
        self.setGoal(0.0,0.5,3.1415)
        self.setGoal(0.0,0.5,3.1415*1.5)
        self.setGoal(0.0,0.5,0)
        

        ###################################
        #上
        #self.setGoal(-0.36,0.28,3.1415*0.25)
        #self.setGoal(0.5,0.0,3.1415)
        #左
        #self.setGoal(0.36,0.28,3.1415*0.75)
        #self.setGoal(0.0,0.5,3.1415*0.75)
        #self.setGoal(0.0,0.5,3.1415)
        #self.setGoal(0.0,0.5,3.1415*1.5)
        #self.setGoal(0.0,0.5,0)


    def strategy(self):
        self.patrol()

def main():
    rospy.init_node('domaeBot')
    node = domaeBot()
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        node.strategy()
        rate.sleep()

if __name__ == '__main__':
    main()
    #rospy.init_node('domaeBot')
    #bot = domaeBot()
    #bot.strategy()
