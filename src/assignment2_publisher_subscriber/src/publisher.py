#!/usr/bin/env python
import rospy
from autominy_msgs.msg import SteeringCommand, SpeedCommand
from std_msgs.msg import Header

rospy.init_node("basic_pub")

steering_publisher = rospy.Publisher("/actuators/steering", SteeringCommand)
speed_publisher = rospy.Publisher("/actuators/speed/", SpeedCommand)

while not rospy.is_shutdown():
    steering_publisher.publish(Header(), 1.0)
    speed_publisher.publish(Header(), 0.3)
    rospy.sleep(0.5)
