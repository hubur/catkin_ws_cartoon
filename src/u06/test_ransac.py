from __future__ import print_function
import sys
import argparse
import numpy as np
import ransac
import matplotlib.pyplot as plt
from matplotlib.image import imread
import cv2


def plot_img(img):
    plt.xticks(np.arange(0, img.shape[1], 1.0))
    plt.yticks(np.arange(0, img.shape[0], 1.0))
    plt.imshow(img)
    plt.show()


if __name__ == "__main__":
    ransac.LineModel([[0, 0], [1, 1]]).sanity_check()
    ransac.LineModel([[0, 1], [1, 0]]).sanity_check()

    img = imread("/Users/mh/kurse/robotik/ransac.png")[:, :, 0]
    print(img.shape)
    ys, xs = np.nonzero(img)

    print(len(ys))
    outliers = list(zip(xs, ys))
    for i in range(3):
        model = ransac.ransac(
            outliers, max_iterations=150, threshold=16, min_inliers=len(xs) / 10
        )
        print(model)
        print(len(model.inliers))
        # print(model.inliers)

        plt.scatter(xs, ys, marker=".")
        plt.scatter(
            [p[0] for p in model.inliers], [p[1] for p in model.inliers], marker="x"
        )
        plt.plot([model.x1, model.x2], [model.y1, model.y2], "r")

        ax = plt.gca()
        ax.set_ylim(ax.get_ylim()[::-1])
        outliers = model.outliers
    plt.show()
