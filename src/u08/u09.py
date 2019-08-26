#!/usr/bin/env python
from __future__ import print_function

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
import numpy as np
from std_msgs.msg import Header
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand, SteeringCommand
from visualization_msgs.msg import Marker
import tf
import math

from nav_msgs.msg import Odometry
from map import Lane, Map
from pid import PID
import math


def publish_point(x, y, marker_id=0, r=0, g=0, b=1):
    marker = Marker()
    marker.id = marker_id
    marker.header.frame_id = "map"
    marker.type = marker.SPHERE
    marker.action = marker.ADD
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.3

    marker.color.a = 1.0
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0

    lookahead_publisher.publish(marker)


def callback(odometry):
    position = odometry.pose.pose.position
    o = odometry.pose.pose.orientation
    euler = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
    # print(".....")
    # print("position", position.x, position.y)
    lane = map.lanes[0]
    car_pos = np.array([position.x, position.y])
    lookahead_point, _ = lane.lookahead_point(car_pos, 50)
    dx = lookahead_point[0] - car_pos[0]
    dy = lookahead_point[1] - car_pos[1]
    publish_point(lookahead_point[0], lookahead_point[1])
    target_yaw = math.atan2(dy, dx)
    measured_yaw = euler[2]
    if abs(target_yaw - measured_yaw) > math.pi:
        d = 2 * math.pi - abs(target_yaw - measured_yaw)
        if target_yaw > measured_yaw:
            d = -d
        print(target_yaw, measured_yaw, d)
        target_yaw = measured_yaw + d
        print(target_yaw)
        # target_yaw = target_yaw % (2*math.pi)
    # print("target yaw  ", target_yaw)
    # print("measured yaw", measured_yaw)
    # print("pid yaw     ", pid_yaw)
    # print(lookahead_point)
    steering_publisher.publish(Header(), target_yaw)


def callback2(msg):
    global pid_yaw
    pid_yaw = msg.value


rospy.init_node("u09")
map = Map()
rospy.Subscriber("/sensors/localization/filtered_map", Odometry, callback, queue_size=1)
rospy.Subscriber(
    "/actuators/steering_normalized", NormalizedSteeringCommand, callback2, queue_size=1
)

steering_publisher = rospy.Publisher("/control/steering", SteeringCommand, queue_size=1)
lookahead_publisher = rospy.Publisher("lookahead_point", Marker, queue_size=1)
rospy.spin()
