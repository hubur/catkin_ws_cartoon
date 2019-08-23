import numpy as np
import sys
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline


def get_splines(points):
    points_u = points[:, 0]
    points_x = points[:, 1]
    points_y = points[:, 2]
    x_spline = CubicSpline(points_u, points_x)
    y_spline = CubicSpline(points_u, points_y)
    return x_spline, y_spline


def distance(point1, point2):
    return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def get_closest_point(point, points, x_spline, y_spline, max_u):
    d_x = (points[:, 1] - point[0]) ** 2
    d_y = (points[:, 2] - point[1]) ** 2
    d = np.sqrt(d_x + d_y)
    closest_two = np.argsort(d)[:2]
    u1 = points[closest_two[0], 0]
    u2 = points[closest_two[1], 0]
    d1 = d[closest_two[0]]
    d2 = d[closest_two[1]]
    last_distance = sys.maxsize
    epsilon = 10 ** (-3)
    while True:
        u_left = min(u1, u2) + abs(u1 - u2) / 2
        u_right = (u_left + max_u / 2) % max_u
        x_left = x_spline(u_left)
        y_left = y_spline(u_left)
        d_left = distance(point, (x_left, y_left))
        x_right = x_spline(u_right)
        y_right = y_spline(u_right)
        d_right = distance(point, (x_right, y_right))
        print(f"u left {u_left} right {u_right} max {max_u}")
        print(f"d left {d_left} right {d_right}")
        print(f"d1 {d1} d2 {d2}")
        print(f"x left {x_left} right {x_right}")
        print(f"y left {y_left} right {y_right}")
        print(f"last_distance {last_distance}")
        print("...")
        plt.scatter((x_left,), (y_left), marker="o")
        plt.scatter((x_right,), (y_right), marker="o")
        if d_left < d_right:
            if d1 < d2:
                u2 = u_left
                d2 = d_left
            else:
                u1 = u_left
                d1 = d_left
            if abs(last_distance - d_left) < epsilon:
                return x_left, y_left
            last_distance = d_left
        else:
            if d1 < d2:
                u2 = u_right
                d2 = d_right
            else:
                u1 = u_right
                d1 = d_right
            if abs(last_distance - d_right) < epsilon:
                return x_right, y_right
            last_distance = d_right


def main():
    lane1 = np.load("/Users/mh/Downloads/lane1.npy")
    lane2 = np.load("/Users/mh/Downloads/lane2.npy")
    points1 = lane1[
        [
            0,
            100,
            150,
            209,
            259,
            309,
            350,
            409,
            509,
            639,
            750,
            848,
            948,
            1028,
            1148,
            1276,
        ],
        :,
    ]
    points2 = lane2[
        [0, 50, 100, 150, 209, 400, 600, 738, 800, 850, 900, 949, 1150, 1300, 1476], :
    ]
    for i, (lane, points) in enumerate(((lane1, points1), (lane2, points2))):
        print("lane", i + 1)
        x_spline, y_spline = get_splines(points)
        lane_u = lane[:, 0]
        max_u = np.max(lane[:, 0])
        min_u = np.min(lane[:, 0])
        print("max min", max_u, min_u)
        point = [3, 2]
        x, y = get_closest_point(point, points, x_spline, y_spline, max_u)
        plt.plot((point[0], x), (point[1], y))
        plt.plot(x_spline(lane_u), y_spline(lane_u))
        plt.scatter(points[:, 1], points[:, 2])
    plt.show()


if __name__ == "__main__":
    main()
