#!/usr/bin/env python
from __future__ import print_function
import math
import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand

BUFFER_SIZE = 8
buffer = []


class GPSData(object):
    def __init__(self, gps_msg):
        secs = gps_msg.header.stamp.secs
        nsecs = gps_msg.header.stamp.nsecs
        q = gps_msg.pose.pose.orientation
        position = gps_msg.pose.pose.position
        yaw = tf.transformations.euler_from_quaternion((q.x, q.y, q.z, q.w))[2]
        self.secs = secs
        self.nsecs = nsecs
        self.position = position
        self.yaw = yaw


def rotation_speed():
    g1 = buffer[-2]
    g2 = buffer[-1]
    dsec = g2.sec - g1.sec
    dnsec = g2.nsec - g1.nsec
    dt = 1000000000 * dsec + dnsec
    dyaw = g2.yaw - g1.yaw
    return dyaw / dt


def steer(target_yaw):
    current = buffer[-1]
    d = target_yaw - current.yaw
    steering_publisher.publish(Header(), 2 * d / math.pi)


def gps_callback(gps_msg):
    gps_data = GPSData(gps_msg)
    position = gps_data.position
    if position.x < 0 or position.x > 5.5 or position.y < 0.5 or position.y > 4.2:
        speed_publisher.publish(Header(), 0)
    else:
        speed_publisher.publish(Header(), 0.5)
    global buffer
    buffer.append(gps_data)
    if len(buffer) > BUFFER_SIZE:
        buffer = buffer[1:]
    steer(0)


rospy.init_node("gps_node")
gps_subscriber = rospy.Subscriber(
    "/communication/gps/12", Odometry, gps_callback, queue_size=1
)
steering_publisher = rospy.Publisher(
    "/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=1
)
speed_publisher = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=1)
rospy.spin()
