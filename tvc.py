import math
import sys
import time
from math import atan2
from ParametricSplineUtil import ParametricSpline, parametric_spline_interpolate
import scipy.integrate as integrate
from scipy import optimize
import numpy as np
import DriveConstants
from Pose import Pose


def arc_length_deriv(x, spline: ParametricSpline):
    derivative = spline.derivative()
    return math.hypot(derivative.spline_x(x), derivative.spline_y(x))


def calculate_arc_length(x, spline: ParametricSpline):
    return integrate.quad(arc_length_deriv, 0, x, args=spline)[0]


def calculate_arc_time(x, spline: ParametricSpline, s):
    return calculate_arc_length(x, spline) - s


class SplineTrajectory(object):

    def __init__(self, spline: ParametricSpline):
        self.spline = spline
        self.functions = len(spline.get_knots()) - 1
        self.arc_length = calculate_arc_length(self.functions, spline)

    def arc_length_to_spline_time(self, s):
        # print(s)
        if s >= self.arc_length:
            return self.functions
        else:
            optimizer = optimize.root_scalar(calculate_arc_time, args=(self.spline, s), bracket=[0, self.functions], method='brentq')
            return optimizer.root

    # spline value at given arc length-- r(t(s))
    def get_spline_value(self, s):
        return self.spline.get(self.arc_length_to_spline_time(s))

    def get_spline_angle(self, s):
        derivative = self.spline.derivative()
        t = self.arc_length_to_spline_time(s)
        return atan2(derivative.spline_y(t), derivative.spline_x(t))


class TrajectoryFollower:
    start_time = 0
    running = False
    follower_pose = Pose(0, 0, 0)

    def __init__(self, trajectory: SplineTrajectory):
        # convert trajectory from arc lengt h to time
        # this is the current implementation of motion profile generation,
        # super simple no acceleration or account for turning speed
        self.trajectory_time = trajectory.arc_length / DriveConstants.max_velocity
        self.trajectory = trajectory

    def arc_length_from_path_time(self, t):
        return t * DriveConstants.max_velocity

    def start(self):
        self.start_time = time.time_ns()
        self.running = True

    def run(self):
        while self.running:
            self.run_async()

    def run_async(self):
        elapsed_time = self.get_elapsed_time()
        position = self.trajectory.get_spline_value(self.arc_length_from_path_time(elapsed_time))
        theta = self.trajectory.get_spline_angle(self.arc_length_from_path_time(elapsed_time))
        self.follower_pose.set_pose(Pose(position[0], position[1], theta))
        if elapsed_time >= self.trajectory_time:
            self.running = False

    def get_elapsed_time(self):
        return (time.time_ns() - self.start_time) / 1000000000


if __name__ == "__main__":
    interpolated_spline = parametric_spline_interpolate([0.0, 30.0, 20.0, 35.0, 0.0], [0.0, 20.0, 5.0, -10.0, 0.0])
    traj = SplineTrajectory(interpolated_spline)
    trajectory_follower = TrajectoryFollower(traj)
    print(traj.arc_length)
    print(traj.get_spline_value(traj.arc_length))
