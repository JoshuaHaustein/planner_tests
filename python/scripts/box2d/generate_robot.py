#! /usr/bin/python

import math


class HolonomicHalfCircle:
    def __init__(self, r, t, f_w, f_l):
        # init points with fingertips
        self.points = [[-r, 0.0, -r + f_w, 0.0, -r + f_w, f_l, -r, f_l],
                        [r - f_w, 0.0, r, 0.0, r, f_l, r - f_w, f_l]]
        # add wings
        left_wing = []
        right_wing = []
        thetas = [i * math.pi / 10.0 for i in range(5)]
        for theta in thetas:
            left_wing.append(r * math.cos(theta + math.pi))
            left_wing.append(r * math.sin(theta + math.pi))
            right_wing.append(r * math.cos(theta + 3.0/2.0 * math.pi + math.pi / 10.0))
            right_wing.append(r * math.sin(theta + 3.0/2.0 * math.pi + math.pi / 10.0))
        left_wing.append(r * math.cos(3.0/2.0 * math.pi - math.pi / 10.0))
        left_wing.append(-r + t)
        left_wing.append(-r + f_w)
        left_wing.append(0.0)
        right_wing.append(r - f_w)
        right_wing.append(0.0)
        right_wing.append(math.cos(3.0 / 2.0 * math.pi + math.pi / 10.0) * r)
        right_wing.append(-r + t)
        bottom_part = [math.cos(3.0 / 2.0 * math.pi - math.pi / 10.0) * r,
                       math.sin(3.0 / 2.0 * math.pi - math.pi / 10.0) * r, math.cos(3.0 / 2.0 * math.pi) * r,
                       math.sin(3.0 / 2.0 * math.pi) * r, math.cos(3.0 / 2.0 * math.pi + math.pi / 10.0) * r,
                       math.sin(3.0 / 2.0 * math.pi + math.pi / 10.0) * r,
                       math.cos(3.0 / 2.0 * math.pi + math.pi / 10.0) * r, -r + t,
                       math.cos(3.0 / 2.0 * math.pi - math.pi / 10.0) * r, -r + t]
        self.points.append(left_wing)
        self.points.append(right_wing)
        self.points.append(bottom_part)

if __name__ == "__main__":
    robot = HolonomicHalfCircle(0.15, 0.08, 0.02, 0.03)
    for elem in robot.points:
        print '-' + str(elem)
