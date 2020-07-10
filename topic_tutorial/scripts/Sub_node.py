#!/usr/bin/env python
import rospy
from std_msgs.msg import String

NAME_TOPIC = '/msgs_talk'
NAME_NODE = 'Sub_node'

def callback(msgs):
    rospy.loginfo(msgs.data)

if __name__ == '__main__':
    rospy.init_node(NAME_NODE,anonymous=True)
    sub = rospy.Subscriber(NAME_TOPIC,String,callback)
    rospy.spin()
