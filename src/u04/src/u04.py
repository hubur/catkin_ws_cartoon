#!/usr/bin/env python
from __future__ import print_function
import math
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

bridge = CvBridge()

tl_c = [110, 130, 250, 280]  # oben links
tr_c = [100, 120, 410, 440]  # oben rechts
ml_c = [160, 190, 230, 260]  # mitte links
mr_c = [140, 160, 430, 460]  # mitte rechts
bl_c = [230, 260, 190, 220]  # unten links
br_c = [190, 250, 470, 550]  # unten rechts
mask_ranges = (bl_c, br_c, ml_c, mr_c, tl_c, tr_c)

mask = np.zeros((480, 640), dtype="uint8")
tl = mask[tl_c[0] : tl_c[1], tl_c[2] : tl_c[3]]  # oben links
tr = mask[tr_c[0] : tr_c[1], tr_c[2] : tr_c[3]]  # oben rechts
ml = mask[ml_c[0] : ml_c[1], ml_c[2] : ml_c[3]]  # mitte links
mr = mask[mr_c[0] : mr_c[1], mr_c[2] : mr_c[3]]  # mitte rechts
bl = mask[bl_c[0] : bl_c[1], bl_c[2] : bl_c[3]]  # unten links
br = mask[br_c[0] : br_c[1], br_c[2] : br_c[3]]  # unten rechts

mask_parts = (bl, br, ml, mr, tl, tr)
for mask_part in mask_parts:
    mask_part[:, :] = 1

# k1, k2, t1, t2, k3 = (0.0, 0.0, 0.0, 0.0, 0.0)
dist_coeffs = np.zeros(5).reshape((5, 1))
# fx, _, cx, _, fy, cy, _, _, _ = (
#    383.7944641113281,
#    0.0,
#    322.3056945800781,
#    0.0,
#    383.7944641113281,
#    241.67051696777344,
#    0.0,
#    0.0,
#    1.0,
# )

camera_matrix = np.array(
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

print(camera_matrix)
print(dist_coeffs)
# exit()

object_points = np.asarray(
    [
        [0.5, 0.2, 0],
        [0.5, -0.2, 0],
        [0.8, 0.2, 0],
        [0.8, -0.2, 0],
        [1.1, 0.2, 0],
        [1.1, -0.2, 0],
    ]
)


def image_callback(image_message):
    cv_image = bridge.imgmsg_to_cv2(image_message, desired_encoding="mono8")
    _, thresh = cv2.threshold(cv_image, 200, 255, cv2.THRESH_BINARY)
    thresh[mask == 0] = 0
    image_points_list = []
    for r in mask_ranges:
        coordinates = np.nonzero(thresh[r[0] : r[1], r[2] : r[3]] != 0)
        ys, xs = coordinates
        y = r[0] + int(math.floor(np.mean(ys)))
        x = r[2] + int(math.floor(np.mean(xs)))
        image_points_list.append((y, x))
    image_points = np.asarray(image_points_list, dtype="float32")
    retval, rvec, tvec = cv2.solvePnP(
        object_points, image_points, camera_matrix, dist_coeffs
    )
    print(
        "\nimage points",
        image_points,
        "\nretval",
        retval,
        "\nrvec",
        rvec,
        "\ntvec",
        tvec,
        sep="\n",
    )
    # print(retval, rvec, tvec)
    for p in image_points_list:
        thresh[p[0] - 1 : p[0] + 1, p[1] - 1 : p[1] + 1] = 0
    binarized_img_publisher.publish(
        bridge.cv2_to_imgmsg(thresh, encoding="passthrough")
    )


rospy.init_node("image_node")

# R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
# P = [383.7944641113281, 0.0, 322.3056945800781, 0.0, 0.0, 383.7944641113281, 241.67051696777344, 0 .0, 0.0, 0.0, 1.0, 0.0]
img_subscriber = rospy.Subscriber(
    "/sensors/camera/infra1/image_rect_raw", Image, image_callback, queue_size=10
)
binarized_img_publisher = rospy.Publisher("/image_binarized", Image, queue_size=10)
rospy.spin()
