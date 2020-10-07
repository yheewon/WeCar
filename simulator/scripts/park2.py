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

class pure_pursuit :
    def __init__(self):
        rospy.init_node('make_path',anonymous=True)
        rospy.Subscriber("path",Path,self.path_callback)
        rospy.Subscriber("odom",Odometry,self.odom_callback)
       # rospy.Subscriber("/acml_pose",PoseWithCovarianceStamped,self.acml_callback)

        self.motor_pub = rospy.Publisher('commands/motor/speed',Float64,queue_size=1)
        self.servo_pub = rospy.Publisher('commands/servo/position',Float64,queue_size=1)
        self.motor_msg = Float64()
        self.servo_msg = Float64()
        self.is_path = False
        self.is_odom = False
        self.is_amcl = False
        self.forward_position = Point()
        self.current_posiiton = Point()
        self.is_look_forward_point = False
        self.vehicle_length = 0.5
        self.Lfd = 0.5
        self.steering = 0

        self.steering_angle_to_servo_gain = -1.2135
        self.steering_angle_to_servo_offset = 0.5304
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.is_path==True and (self.is_odom == True or self.is_amcl == True):
                vehicle_position = self.current_posiiton
                rotated_point = Point()
                self.is_look_forward_point = False

                for num,i in enumerate(self.path.poses):
                    path_point = i.pose.position
                    dx = path_point.x - vehicle_position.x
                    dy = path_point.y - vehicle_position.y
                    rotated_point.x = cos(self.vehicle_yaw)*dx + sin(self.vehicle_yaw)*dy
                    rotated_point.y = sin(self.vehicle_yaw)*dx - cos(self.vehicle_yaw)*dy

                    if rotated_point.x>0 :
                        dis = sqrt(pow(rotated_point.x,2)+pow(rotated_point.y,2))
                        if dis >= self.Lfd:
                            self.forward_point = path_point
                            self.is_look_forward_point=True

                            break
                
                theta =-atan2(rotated_point.y,rotated_point.x)
                if self.is_look_forward_point:
                    self.steering = atan2((2*self.vehicle_length*sin(theta)),self.Lfd)
                    print(self.steering*180/pi)
                    self.motor_msg.data = 1000

                else :
                    # controll = simple_controller()
                    self.steering = 0
                    self.motor_msg.data = 0
                    con = simple_controller()
                    
                    
                    

                self.steering_commands = (self.steering_angle_to_servo_gain*self.steering)+self.steering_angle_to_servo_offset
                self.servo_msg.data = self.steering_commands

                self.servo_pub.publish(self.servo_msg)
                self.motor_pub.publish(self.motor_msg)

            rate.sleep()
    def path_callback(self,msg):
        self.is_path = True
        self.path = msg
    def odom_callback(self,msg):
        self.is_odom = True
        odom_quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw = euler_from_quaternion(odom_quaternion)
        self.current_posiiton.x = msg.pose.pose.position.x
        self.current_posiiton.y = msg.pose.pose.position.y
    def acml_callback(self,msg ):
        self.is_amcl = True
        amcl_quaternion = (msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
        _,_,self.vehicle_yaw = euler_from_quaternion(amcl_quaternion)
        self.current_posiiton.x = msg.pose.pose.position.x
        self.current_posiiton.y = msg.pose.pose.position.y

class simple_controller:
    def __init__(self):
        # rospy.init_node('simple_controller',anonymous=True)
        rospy.Subscriber("/scan",LaserScan,self.laser_callback)
        self.motor_pub = rospy.Publisher('commands/motor/speed',Float64,queue_size=1)
        self.servo_pub = rospy.Publisher('commands/servo/position',Float64,queue_size=1)
        self.pcd_pub = rospy.Publisher('laser2pcd',PointCloud,queue_size=1)
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
        # count_right = 0
        count_left = 0
       
        for point in pcd.points:
            # if point.x >0 and point.x<2.5 and point.y>0 and point.y<1:
            #     count_right = count_right +1
            
            # left
            if point.x >0 and point.x< 0.5 and point.y>0 and point.y<1: 
                count_left = count_left +1
        print(count_left)
        
        # if count_left>10 and count_right>10:
        #     if count_left > count_right :
        #         while count_right > 10 :
        #           servo_value = 0.85
        #           count_right = count_right -1 
        #     elif count_left < count_right:
        #          while count_left > 10 :
        #            servo_value = 0.15
        #            count_left = count_left -1 

        # if count_left >= 18 :
        #     while count_left > 18 :
        #         servo_value = 0.15
        #         count_left = count_left -1 

        # elif count_right >= 18  :
        #    while count_right > 18 :
        #         servo_value = 0.85
        #         count_right = count_right -1 
        if count_left >= 10 :
            servo_value = 0.5304
            count_left = 0
            
        else : 
            servo_value = 0.15

        
        motor_msg.data = 2000


        # print(count_right)
        # print(count_left)
        print("------")
        self.motor_pub.publish(motor_msg)
        self.servo_pub.publish(servo_value)
        self.pcd_pub.publish(pcd)

if __name__ == '__main__':
    try :
        test_track = pure_pursuit()
    except rospy.ROSInterruptException:
        pass

        
