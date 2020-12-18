#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rospkg
from sensor_msgs.msg import LaserScan,PointCloud,Imu
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from laser_geometry import LaserProjection
from math import cos,sin,pi,sqrt,pow,atan2
from geometry_msgs.msg import Point32,PoseStamped,Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry,Path

import tf
from tf.transformations import euler_from_quaternion,quaternion_from_euler

class simple_controller:
    def __init__(self):
        rospy.init_node('simple_controller',anonymous=True)
        rospy.Subscriber("/scan",LaserScan,self.laser_callback)
        self.motor_pub = rospy.Publisher('commands/motor/speed',Float64,queue_size=1)
        self.servo_pub = rospy.Publisher('commands/servo/position',Float64,queue_size=1)
        self.pcd_pub = rospy.Publisher('laser2pcd',PointCloud,queue_size=1)
        self. switch = 0 
        self. left = 0 
        self. stop = 1
        print("simple controller")

        while not rospy.is_shutdown():
            rospy.spin()


    def laser_callback(self,msg):
        pcd = PointCloud()
        motor_msg = Float64()
        servo_value = Float64()
        pcd.header.frame_id = msg.header.frame_id
        angle = 0

        for r in msg.ranges:
            tmp_point= Point32()
            tmp_point.x = r*cos(angle)
            tmp_point.y = r*sin(angle)
            angle = angle + (1.0/180*pi)
            if r<2 :
                pcd.points.append(tmp_point)
        count_right = 0
        count = 0 
        count_left = 0
        back = 0 
       
        for point in pcd.points:
            if point.x >0 and point.x< 1: 
                count = count + 1

            if point.x >0 and point.x<0.3 and point.y>-1 and point.y<0:
                count_right = count_right +1
            
            # left
            if point.x >0 and point.x< 0.3 and point.y>0 and point.y<1: 
                count_left = count_left +1

            

        print("left : " , count_left)
        print("right : " , count_right)
        print("count : ", count)
        print("switch : ", self.switch)

  
        if self.switch == 0 :
            if count_left >= 10 :
                servo_value = 0.5304
                # if back == 1 : 
                #     self.switch = 1
                #     print("##############################################################################################")
            
            
            elif count_left < 10 : 
                servo_value = 0.15
                self. left = 1

            motor_msg.data = 1000

            if count >= 60 and self.left == 1 :
                if count_left > count_right : 
                    servo_value = 0.15
                else : 
                    servo_value = 0.85
                  
                servo_value = 0.5304
                motor_msg.data = -1000 
                back = 1
                
                print("backbackkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk")
                self.switch = 1

            # if count < 60 and back == 1 : 
            #     self.switch = 1
            #     print("##############################################################################################")
            
        elif self.switch == 1:
            print("*****************")
            if count_left >= 10 : 
                servo_value = 0.5304

            else : 
                servo_value = 0.15

            motor_msg.data = 1000

            if count >= 70 : 
                motor_msg.data = 0
                servo_value = 0.5304

            
            
            # if count_left>10 and count_right>10:
            #     if count_left > count_right :
            #         while count_right > 10 :
            #             servo_value = 0.85
            #             count_right = count_right -1 
            # elif count_left < count_right:
            #     while count_left > 10 :
            #        servo_value = 0.15
            #        count_left = count_left -1 

            # if count_left >= 18 :
            #     while count_left > 18 :
            #         servo_value = 0.15
            #         count_left = count_left -1 

            # elif count_right >= 18  :
            #     while count_right > 18 :
            #         servo_value = 0.85
            #         count_right = count_right -1 

            # else : 
            #     servo_value = 0.5304

        # motor_msg.data = 1000

             
        
        print("------")
        self.motor_pub.publish(motor_msg)
        self.servo_pub.publish(servo_value)
        self.pcd_pub.publish(pcd)

if __name__ == '__main__':
    try :
        test_track = simple_controller()
    except rospy.ROSInterruptException:
        pass
