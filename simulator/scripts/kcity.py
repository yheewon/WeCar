#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from sensor_msgs.msg import LaserScan,PointCloud,Imu
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos,sin,pi,sqrt,pow
from geometry_msgs.msg import Point32, PoseStamped
from nav_msgs.msg import Odometry, Path
import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler
import glob

class kcity_pub :

    def __init__(self):
        # rospy.init_node("path_pub", anonymous=True)
        rospy.init_node("kcity_pub",anonymous=True)

        self.kcity_pub = rospy.Publisher('/pointcloud',PointCloud,queue_size=1)
        self.kcity_msg = PointCloud()
        self.kcity_msg.header.frame_id = '/map'
        # self.path_pub = rospy.Publisher('/path',Path,queue_size=1)
        # self.path_msg=Path()
        # self.path_msg.header.frame_id='/odom'
     
        rospack=rospkg.RosPack()
        pkg_path=rospack.get_path('simulator')
        c_list = glob.glob(pkg_path+'/kcity_PM0138/*.csv')
        # full_path=pkg_path+'/kcity_PM0138'+'/A1LANE_CenterLine_0001.csv'
        for f_line in c_list:

            self.f=open(f_line,'r')
            lines=self.f.readlines()
            i = 1
            for line in lines :
                if i > 8 :
                    tmp=line.split()
                    read_pose=Point32()
                    read_pose.x=float(tmp[0])-302459.942
                    read_pose.y=float(tmp[1])-4122635.537
                    read_pose.z=float(tmp[2])-28.989999771118164

                    self.kcity_msg.points.append(read_pose)
                i = i +1
            
            self.f.close()

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.kcity_pub.publish(self.kcity_msg)
            rate.sleep()
               
if __name__ == '__main__':
    try:
        test_track=kcity_pub()
    except rospy.ROSInterruptException:
        pass