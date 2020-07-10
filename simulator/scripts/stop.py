#!/usr/bin/env python
# _*_ coding: utf-8 _*_

import rospy
from sensor_msgs.msg import LaserScan,PointCloud
from std_msgs.msg import Float64
from vesc_msgs.msg import VescStateStamped
from math import cos,sin,pi
from geometry_msgs.msg import Point32

class simple_controller:
    def __init__(self):
        rospy.init_node('simple_controller',anonymous=True)
        rospy.Subscriber("/scan",LaserScan,self.laser_callback)
        self.motor_pub = rospy.Publisher('commands/motor/speed',Float64,queue_size=1)
        self.servo_pub = rospy.Publisher('commands/servo/position',Float64,queue_size=1)
        self.pcd_pub = rospy.Publisher('laser2pcd',PointCloud,queue_size=1)

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
        count_left = 0
        for point in pcd.points:
            if point.x >0 and point.x<2 and point.y>0 and point.y<1:
                count_right = count_right +1

            if point.x >0 and point.x<2 and point.y>-1 and point.y<0:
                count_left = count_left +1
        
        if count_left>10 and count_right>10:
            if count_left > count_right :
                while count_right > 10 :
                  servo_value = 0.85
                  count_right = count_right -1 
            elif count_left < count_right:
                 while count_left > 10 :
                   servo_value = 0.15
                   count_left = count_left -1 

        if count_left > 18 :
            while count_left > 18 :
                servo_value = 0.15
                count_left = count_left -1 

        elif count_right > 18  :
           while count_right > 18 :
                servo_value = 0.85
                count_right = count_right -1 
        else : 
            servo_value = 0.5304
        motor_msg.data = 4000


        print(count_right)
        print(count_left)
        print("------")
        self.motor_pub.publish(motor_msg)
        self.servo_pub.publish(servo_value)
        self.pcd_pub.publish(pcd)
  
 


if __name__ == '__main__':
    try:
        test_track = simple_controller()

    except rospy.ROSInterruptException:
        pass