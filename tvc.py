import time
from math import atan2
from ParametricSplineUtil import ParametricSpline, parametric_spline_interpolate
import scipy.integrate as integrate
import numpy as np
import DriveConstants
from Pose import Pose


def integrand(x, spline):
    return np.hypot(spline.spline_x.derivative()(x), spline.spline_y.derivative()(x))


def find_arc_length(spline: ParametricSpline):
    knots = spline.get_knots()
    upper_bound = knots[len(knots) - 1]
    return integrate.quad(integrand, 0, upper_bound, args=spline)[0]


class SplineTrajectory(object):

    def __init__(self, spline: ParametricSpline):
        self.spline = spline
        self.arc_length = find_arc_length(spline)
        self.parameterize_scale = (len(spline.get_knots()) - 1) / self.arc_length

    def get_spline_value(self, t):
        return self.spline.get(t * self.parameterize_scale)

    def get_spline_angle(self, t):
        derivative_spline = self.spline.derivative()
        return atan2(derivative_spline.spline_y(t * self.parameterize_scale),
                     derivative_spline.spline_x(t * self.parameterize_scale))


class TrajectoryFollower:
    start_time = 0
    running = False
    follower_pose = Pose(0, 0, 0)

    def __init__(self, trajectory: SplineTrajectory):
        # convert trajectory from arc length to time
        # this is the current implementation of motion profile generation,
        # super simple no acceleration or account for turning speed
        self.trajectory_time = trajectory.arc_length / DriveConstants.max_velocity
        trajectory.parameterize_scale = (len(trajectory.spline.get_knots()) - 1) / self.trajectory_time
        self.trajectory = trajectory

    def start(self):
        self.start_time = time.time_ns()
        self.running = True

    def run(self):
        while self.running:
            self.run_async()

    def run_async(self):
        elapsed_time = self.get_elapsed_time()
        position = self.trajectory.get_spline_value(elapsed_time)
        theta = self.trajectory.get_spline_angle(elapsed_time)
        self.follower_pose.set_pose(Pose(position[0], position[1], theta))
        if elapsed_time >= self.trajectory_time:
            self.running = False

    def get_elapsed_time(self):
        return (time.time_ns() - self.start_time) / 1000000000


if __name__ == "__main__":
    # simple test to make sure shit isnt broken
    interpolated_spline = parametric_spline_interpolate([0.0, 15.0, 20.0], [0.0, 10.0, 5.0])
    traj = SplineTrajectory(interpolated_spline)
    trajectory_follower = TrajectoryFollower(traj)
    trajectory_follower.start()
    time.sleep(1)
    trajectory_follower.run_async()
    print(trajectory_follower.follower_pose.x)
