#!/usr/bin/env python
import rospy
from autominy_msgs.msg import Speed

def callback(raw_msg):
    print raw_msg


rospy.init_node("basic_sub")

rospy.Subscriber("/sensors/speed", Speed, callback)

rospy.spin()
