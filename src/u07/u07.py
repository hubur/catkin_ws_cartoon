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


def rotation_speed_diff():
    def speed(i):
        g1 = buffer[-2 - i]
        g2 = buffer[-1 - i]
        dsec = g2.secs - g1.secs
        dnsec = g2.nsecs - g1.nsecs
        dt = 1000000000 * dsec + dnsec
        dyaw = g2.yaw - g1.yaw
        return dyaw / dt

    return speed(1) - speed(0)


def integrate_error():
    return sum([(p1.yaw - p2.yaw) for p1, p2 in zip(buffer[:-1], buffer[1:])])


def steer(target_yaw):
    current = buffer[-1]
    d = target_yaw - current.yaw
    k_p = 12 / math.pi
    k_d = 10 ** 8
    k_i = 40 / len(buffer)
    rsd = rotation_speed_diff()
    ie = integrate_error()
    u = 0.3 * (k_p * d + k_d * rsd + k_i * ie)
    print(d, rsd, ie, u)
    steering_publisher.publish(Header(), u)


def gps_callback(gps_msg):
    gps_data = GPSData(gps_msg)
    position = gps_data.position
    global buffer
    buffer.append(gps_data)
    if len(buffer) > BUFFER_SIZE:
        buffer = buffer[1:]
    if position.x < 0 or position.x > 5.5 or position.y < 0.5 or position.y > 4.2:
        speed_publisher.publish(Header(), 0)
    else:
        steer(0)
        speed_publisher.publish(Header(), 0.5)


rospy.init_node("gps_node")
gps_subscriber = rospy.Subscriber(
    "/communication/gps/12", Odometry, gps_callback, queue_size=1
)
steering_publisher = rospy.Publisher(
    "/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=1
)
speed_publisher = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=1)
rospy.spin()
