import numpy as np
import sys
from scipy.interpolate import CubicSpline

LOOK_AHEAD_DISTANCE = 1


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


def main(car_position= [6, 4], do_plots=False):
    lane1 = np.load("lane1.npy")
    lane2 = np.load("lane2.npy")
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
    splines = []
    closest_points = []
    carrot_points = []
    for i, (lane, points) in enumerate(((lane1, points1), (lane2, points2))):
        print("lane", i + 1)
        x_spline, y_spline = get_splines(points)
        lane_u = lane[:, 0]
        max_u = np.max(lane[:, 0])
        min_u = np.min(lane[:, 0])
        print("max min", max_u, min_u)
        x, y = get_closest_point(car_position, points, x_spline, y_spline, max_u)
        vec_lane_to_car = np.array([car_position[0] - x, car_position[1] - y])
        vec_to_carrot = rotate90(vec_lane_to_car) / np.linalg.norm(vec_lane_to_car)
        vec_to_carrot *= LOOK_AHEAD_DISTANCE
        next_to_carrot_point = vec_to_carrot + np.array([x, y])
        carrot_x, carrot_y = get_closest_point(
            next_to_carrot_point, points, x_spline, y_spline, max_u
        )
        splines.append((x_spline(lane_u), y_spline(lane_u)))
        closest_points.append((x,y))
        carrot_points.append((carrot_x, carrot_y))
        if do_plots:
            import matplotlib.pyplot as plt
            plt.scatter(carrot_x, carrot_y, marker="D")
            plt.scatter(next_to_carrot_point[0], next_to_carrot_point[1], marker="x")
            plt.plot((car_position[0], x), (car_position[1], y))
            plt.plot(x_spline(lane_u), y_spline(lane_u))
            # plt.scatter(points[:, 1], points[:, 2])
    if do_plots:
        plt.show()
    return splines, closest_points, carrot_points


ROTATION_MATRIX = np.array([[0, 1], [-1, 0]])


def rotate90(vec):
    return np.dot(ROTATION_MATRIX, vec)


if __name__ == "__main__":
    main(do_plots=True)
