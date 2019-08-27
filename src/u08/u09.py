#!/usr/bin/env python
from __future__ import print_function

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
import numpy as np
from std_msgs.msg import Header, Int32
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand, SteeringCommand
from visualization_msgs.msg import Marker
import tf
import math

from nav_msgs.msg import Odometry
from map import Lane, Map
from pid import PID
import math

LOOKAHEAD_DISTANCE = 0.4
lane_no = 1


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


def curvature(point, param, lane):
    p0 = lane.interpolate(param - 0.1)
    p1 = point
    p2 = lane.interpolate(param + 0.1)

    angle1 = math.atan2(p0[0] - p1[0], p0[1] - p1[1])
    angle2 = math.atan2(p1[0] - p2[0], p1[1] - p2[1])
    print("angles", angle1, angle2, param, point)
    d = abs(angle1 - angle2)
    if d > math.pi:
        d = 2 * math.pi - d
    return 1 - d / (2 * math.pi)


def callback(odometry):
    position = odometry.pose.pose.position
    o = odometry.pose.pose.orientation
    euler = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
    lane = map.lanes[lane_no]
    car_pos = np.array([position.x, position.y])
    print(car_pos)
    lookahead_point, param = lane.lookahead_point(car_pos, LOOKAHEAD_DISTANCE)
    dx = lookahead_point[0] - car_pos[0]
    dy = lookahead_point[1] - car_pos[1]
    publish_point(lookahead_point[0], lookahead_point[1])
    target_yaw = math.atan2(dy, dx)
    measured_yaw = euler[2]
    d = abs(target_yaw - measured_yaw)
    print(d, end=" ")
    if d > math.pi:
        d = 2 * math.pi - d
        if target_yaw > measured_yaw:
            d = -d
        print(target_yaw, measured_yaw, d)
        target_yaw = measured_yaw + d
        print(target_yaw)
        print("edge case: %f" % d)
    else:
        print("normal")
    steering_publisher.publish(Header(), target_yaw)
    curve = curvature(lookahead_point, param, lane)
    print("curve", curve)
    speed_publisher.publish(Header(), min(0.5 * curve, 0.5))


def lane_switch_callback(msg):
    global lane_no
    lane_no = msg.data


rospy.init_node("u09")
map = Map()
rospy.Subscriber("/sensors/localization/filtered_map", Odometry, callback, queue_size=1)
rospy.Subscriber("/lane_no", Int32, lane_switch_callback, queue_size=1)

steering_publisher = rospy.Publisher("/control/steering", SteeringCommand, queue_size=1)
speed_publisher = rospy.Publisher("/control/speed", SpeedCommand, queue_size=1)
lookahead_publisher = rospy.Publisher("lookahead_point", Marker, queue_size=1)
rospy.spin()
