#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
import rospy
import rosparam
#import tf2_ros
import tf
import geometry_msgs.msg
from obstacle_detector.msg import Obstacles
#
class enemy_detect():
    def __init__(self):
        self.obstacle_sub = rospy.Subscriber('/tracked_obstacles', Obstacles, self.obstacle_callback)
        self.enemy_pub = tf.TransformBroadcaster()

    def obstacle_callback(self, data):
        for i in range(len(data.circles)):
            ob_x = data.circles[i].center.x
            ob_y = data.circles[i].center.y
            self.judge_enemy(ob_x, ob_y)

    def judge_enemy(self,ob_x,ob_y):
        trans_ob_x = ob_x*(math.cos(math.radians(-45))-ob_y*math.sin(math.radians(-45)))
        trans_ob_y = ob_x*(math.sin(math.radians(-45))+ob_y*math.cos(math.radians(-45)))

        if (trans_ob_x < -1.1 or trans_ob_x > 1.1) or (trans_ob_y < -1.1 or trans_ob_y >1.1):
            return 0

        if (ob_x > -0.1 and ob_x < 0.1) and (ob_y > -0.1 and ob_y < 0.1):
            return 0
        elif (ob_x < 0.6 and ob_x > 0.4) and (ob_y > 0.4 and ob_y < 0.6):
            return 0
        elif (ob_x > -0.6 and ob_x < -0.4) and (ob_y > 0.4 and ob_y < 0.6):
            return 0
        elif (ob_x > 0.4 and ob_x < 0.6) and (ob_y > -0.6 and ob_y < -0.4):
            return 0
        elif (ob_x > -0.6 and ob_x < -0.4) and (ob_y > -0.6 and ob_y < -0.4):
            return 0
        #print "x = ",ob_x,"y = ",ob_y
        print "t_x = ",trans_ob_x,"t_y = ",trans_ob_y
        self.enemy_pub.sendTransform((ob_x,ob_y,0),
                                tf.transformations.quaternion_from_euler(0, 0, 0),
                                rospy.Time.now(),"d_enemy","map")
        return 0

        
if __name__ == '__main__':
    rospy.init_node('enemy_detect')
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        e_detect = enemy_detect()
        rate.sleep()
    
