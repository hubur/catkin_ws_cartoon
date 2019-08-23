#!/usr/bin/env python
from __future__ import print_function

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PointStamped
import lookahead

rospy.init_node("line_pub_example")
pub_line_min_dist = rospy.Publisher("~line_min_dist", Marker, queue_size=1)
rospy.loginfo("Publishing example line")

marker_id = 1


def get_marker_id():
    global marker_id
    marker_id += 1
    return marker_id


def publish_point(x, y, marker_id, r=0, g=0, b=1):
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

    pub_line_min_dist.publish(marker)


def mark_clicked_point(click):
    print(click)
    publish_point(click.point.x, click.point.y, marker_id=1)
    return
    marker = Marker()
    marker.id = 1234
    marker.header.frame_id = "map"
    marker.type = marker.SPHERE
    marker.action = marker.ADD

    # marker scale
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.3

    # marker color
    marker.color.a = 1.0
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0

    # marker orientaiton
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0

    # marker position
    marker.pose.position.x = click.point.x
    marker.pose.position.y = click.point.y
    marker.pose.position.z = click.point.z

    # Publish the Marker
    pub_line_min_dist.publish(marker)


def show_lanes(splines):
    for i, spline in enumerate(splines):
        marker = Marker()
        marker.id = i
        marker.header.frame_id = "map"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        marker.scale.x = 0.03
        marker.scale.y = 0.03
        marker.scale.z = 0.03

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.points = []
        for i in range(len(spline[0])):
            point = Point()
            point.x = spline[0][i]
            point.y = spline[1][i]
            point.z = 0.0
            marker.points.append(point)
        pub_line_min_dist.publish(marker)


def callback(click):
    mark_clicked_point(click)
    splines, closest_points, carrot_points = lookahead.main(
        (click.point.x, click.point.y)
    )
    show_lanes(splines)
    i = 3
    for p in closest_points:
        publish_point(p[0], p[1], r=0, g=1, b=0, marker_id=i)
        i += 1
    for p in carrot_points:
        publish_point(p[0], p[1], r=1, g=0, b=0, marker_id=i)
        i += 1


click_subscriber = rospy.Subscriber(
    "/clicked_point", PointStamped, callback, queue_size=1
)
rospy.spin()
