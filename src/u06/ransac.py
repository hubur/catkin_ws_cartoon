from __future__ import print_function
import math
import random
import sys
import numpy as np
import cv2


class LineModel(object):
    def __init__(self, inliers=None):
        if not inliers:
            self.x1 = self.y1 = self.x2 = self.y2 = self.a = self.b = self.c = 0
            self.error = sys.maxsize
            self.inliers = []
            return
        self.x1, self.y1 = inliers[0]
        self.x2, self.y2 = inliers[1]
        self.inliers = inliers
        self.a = self.y1 - self.y2
        self.b = self.x2 - self.x1
        self.c = -(self.x2 * self.y1 - self.x1 * self.y2)
        self.error = 0.0

    def sanity_check(self):
        d = _distance((self.x1, self.y1), self)
        assert d == 0, (d, self.a, self.b, self.c)
        d = _distance((self.x2, self.y2), self)
        assert d == 0, (d, self.a, self.b, self.c)

    def __repr__(self):
        return (
            "LineModel x1: %d, y1: %d, x2: %d, y2: %d, a: %d, b: %d, c: %d, error: %f"
            % (self.x1, self.y1, self.x2, self.y2, self.a, self.b, self.c, self.error)
        )


def _distance(point, line):
    x = point[0]
    y = point[1]
    return abs(line.a * x + line.b * y + line.c) / math.sqrt(line.a ** 2 + line.b ** 2)


def _random_point(xs, ys, num_points):
    i = random.randint(0, num_points - 1)
    return (xs[i], ys[i])


def ransac(xs, ys, max_iterations, threshold, min_inliers):
    assert len(xs) == len(ys)

    best_model = LineModel()
    for i in range(max_iterations):
        model = LineModel([_random_point(xs, ys, len(xs)) for _ in range(2)])
        model.sanity_check()
        for p in zip(xs, ys):
            d = _distance(p, model)
            if d < threshold:
                model.inliers.append(p)
                model.error += d
        if len(model.inliers) > min_inliers and model.error < best_model.error:
            best_model = model

    return best_model
