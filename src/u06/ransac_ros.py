#!/usr/bin/env python
from __future__ import print_function
import math
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import ransac


def get_mask_ranges():
    l1 = [200, 360, 0, 640]
    return (l1,)


def get_mask(mask_ranges):
    mask = np.zeros((480, 640), dtype="uint8")
    for r in mask_ranges:
        mask[r[0] : r[1], r[2] : r[3]] = 1
    return mask


def calculate_image_points(image_message):
    cv_image = BRIDGE.imgmsg_to_cv2(image_message, desired_encoding="mono8")
    _, thresh = cv2.threshold(cv_image, 100, 255, cv2.THRESH_BINARY)
    thresh[MASK == 0] = 0
    image_points_list = []
    for r in MASK_RANGES:
        ys, xs = np.nonzero(thresh[r[0] : r[1], r[2] : r[3]] != 0)
        outliers = list(zip(xs, ys))
        for i in range(4):
            model = ransac.ransac(outliers, 500, 16, 300)
            p1 = model.inliers[0]
            p2 = model.inliers[1]
            cv2.line(
                cv_image,
                (p1[0] + r[2], p1[1] + r[0]),
                (p2[0] + r[2], p2[1] + r[0]),
                0,
                4,
            )
            print(model)
            m = -model.a / model.b
            b = -model.c / model.b
            print("m: %f, b: %f" % (m, b))
            outliers = model.outliers

    #binarized_img_publisher.publish(
    #    BRIDGE.cv2_to_imgmsg(cv_image, encoding="passthrough")
    #)
    line_img_publisher.publish(BRIDGE.cv2_to_imgmsg(cv_image, encoding="passthrough"))
    for p in image_points_list:
        thresh[p[0] - 1 : p[0] + 1, p[1] - 1 : p[1] + 1] = 0
    binarized_img_publisher.publish(
       BRIDGE.cv2_to_imgmsg(thresh, encoding="passthrough")
    )
    return image_points_list


def image_callback(image_message):
    image_points_list = calculate_image_points(image_message, publish_thresh=True)
    return
    image_points = np.asarray(image_points_list, dtype="float32")
    _, rvec, tvec = cv2.solvePnP(
        OBJECT_POINTS, image_points, CAMERA_MATRIX, DIST_COEFFS
    )
    rotation_matrix, _ = cv2.Rodrigues(rvec)

    H = np.concatenate((rotation_matrix, tvec), axis=1)
    H = np.concatenate((H, np.array([0, 0, 0, 1]).reshape(1, 4)))
    inverse = np.linalg.inv(H)
    projected_points, _ = cv2.projectPoints(
        OBJECT_POINTS, rvec, tvec, CAMERA_MATRIX, DIST_COEFFS
    )


MASK_RANGES = get_mask_ranges()
MASK = get_mask(MASK_RANGES)
BRIDGE = CvBridge()

rospy.init_node("image_node")

img_subscriber = rospy.Subscriber(
    "/sensors/camera/infra1/image_rect_raw", Image, image_callback, queue_size=10
)
binarized_img_publisher = rospy.Publisher("/image_binarized", Image, queue_size=10)
line_img_publisher = rospy.Publisher("/image_lines", Image, queue_size=10)
rospy.spin()
