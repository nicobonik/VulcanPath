from scipy.interpolate import CubicSpline, PPoly
import numpy as np
from typing import List


class ParametricSpline(object):

    def __init__(self, spline_x: PPoly, spline_y: PPoly):
        self.spline_x = spline_x
        self.spline_y = spline_y

    def get(self, t):
        return np.array([self.spline_x(t), self.spline_y(t)])

    def derivative(self):
        return ParametricSpline(self.spline_x.derivative(), self.spline_y.derivative())

    def get_knots(self):
        return self.spline_y.x


def parametric_spline_interpolate(x: List[float], y: List[float]) -> ParametricSpline:
    t = np.linspace(0, len(x) - 1, len(x))
    interpolated_spline_x = CubicSpline(t, x)
    interpolated_spline_y = CubicSpline(t, y)
    return ParametricSpline(interpolated_spline_x, interpolated_spline_y)
