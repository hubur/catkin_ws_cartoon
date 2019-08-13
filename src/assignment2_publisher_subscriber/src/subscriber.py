#!/usr/bin/env python
from __future__ import print_function
import rospy
from autominy_msgs.msg import Speed

rospy.init_node("basic_sub")

rospy.Subscriber("/sensors/speed", Speed, print)

rospy.spin()
