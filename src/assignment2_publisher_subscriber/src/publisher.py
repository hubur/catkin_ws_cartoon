#!/usr/bin/env python
import math
import rospy
from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand
from std_msgs.msg import Header
from rospy.timer import Rate

rospy.init_node("basic_pub")

steering_publisher = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=1)
speed_publisher = rospy.Publisher("/actuators/speed/", SpeedCommand, queue_size=1)

rate = Rate(hz=100, reset=True)
while not rospy.is_shutdown():
    steering_publisher.publish(Header(), 1.0)
    speed_publisher.publish(Header(), 0.3)
    rate.sleep()
