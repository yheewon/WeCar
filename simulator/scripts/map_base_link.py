#!/usr/bin/env python
#-*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan,PointCloud
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos, sin, pi, atan
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class base_link:
    def __init__(self):
        rospy.init_node("base_link",anonymous=True)

        rospy.Subscriber("morai_odom", Odometry, self.odom_callback)

        self.x = 0
        self.y = 0
        self.z = 0
       

        while not rospy.is_shutdown():
            br=tf.TransformBroadcaster()
            br.sendTransform((self.x,self.y,self.z),
                (0,0,1,0),
                rospy.Time.now(),
                "base_link",
                "map")

            
            


    def odom_callback(self,msg):
        self.x=msg.pose.pose.position.x
        self.y=msg.pose.pose.position.y
        self.z=msg.pose.pose.position.z



if __name__=='__main__':
    try:
        test_track=base_link()
    except rospy.ROSInterruptException:
        pass
       

