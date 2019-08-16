#!/usr/bin/env python
from __future__ import print_function
import math
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


def get_mask_ranges():
    tl = [110, 130, 250, 280]  # oben links
    tr = [100, 120, 410, 440]  # oben rechts
    ml = [160, 190, 230, 260]  # mitte links
    mr = [140, 160, 430, 460]  # mitte rechts
    bl = [230, 260, 190, 220]  # unten links
    br = [190, 250, 470, 550]  # unten rechts
    return bl, br, ml, mr, tl, tr


def get_mask(mask_ranges):
    mask = np.zeros((480, 640), dtype="uint8")
    for r in mask_ranges:
        mask[r[0] : r[1], r[2] : r[3]] = 1
    return mask


def calculate_image_points(image_message, publish_thresh):
    cv_image = BRIDGE.imgmsg_to_cv2(image_message, desired_encoding="mono8")
    _, thresh = cv2.threshold(cv_image, 200, 255, cv2.THRESH_BINARY)
    thresh[MASK == 0] = 0
    image_points_list = []
    for r in MASK_RANGES:
        ys, xs = np.nonzero(thresh[r[0] : r[1], r[2] : r[3]] != 0)
        y = r[0] + int(math.floor(np.mean(ys)))
        x = r[2] + int(math.floor(np.mean(xs)))
        image_points_list.append((y, x))
    if publish_thresh:
        for p in image_points_list:
            thresh[p[0] - 1 : p[0] + 1, p[1] - 1 : p[1] + 1] = 0
        binarized_img_publisher.publish(
            BRIDGE.cv2_to_imgmsg(thresh, encoding="passthrough")
        )
    return image_points_list


def image_callback(image_message):
    image_points_list = calculate_image_points(image_message, publish_thresh=True)
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
    print(
        "\nimage points",
        image_points,
        "\nprojected points",
        projected_points,
        "\nrvec",
        rvec,
        "\ntvec",
        tvec,
        np.linalg.norm(tvec),
        "\nR",
        rotation_matrix,
        "\nH",
        H,
        "\ninverse",
        inverse,
        sep="\n",
    )


MASK_RANGES = get_mask_ranges()
MASK = get_mask(MASK_RANGES)
BRIDGE = CvBridge()
DIST_COEFFS = np.zeros(5).reshape((5, 1))
CAMERA_MATRIX = np.array(
    [
        383.7944641113281,
        0.0,
        322.3056945800781,
        0.0,
        383.7944641113281,
        241.67051696777344,
        0.0,
        0.0,
        1.0,
    ]
).reshape((3, 3))
OBJECT_POINTS = np.asarray(
    [
        [0.5, 0.2, 0],
        [0.5, -0.2, 0],
        [0.8, 0.2, 0],
        [0.8, -0.2, 0],
        [1.1, 0.2, 0],
        [1.1, -0.2, 0],
    ]
)

rospy.init_node("image_node")

img_subscriber = rospy.Subscriber(
    "/sensors/camera/infra1/image_rect_raw", Image, image_callback, queue_size=10
)
binarized_img_publisher = rospy.Publisher("/image_binarized", Image, queue_size=10)
rospy.spin()
