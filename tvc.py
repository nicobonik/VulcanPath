import time


class TimeVariantController:
    start_time = 0
    max_velocity = 0.0
    max_accel = 0.0
    max_angular_velocity = 0.0

    def __init__(self):
        return

    def start(self):
        self.start_time = time.time_ns()


class SimpleTrajectory:
    max_velocity = 0.0
    max_accel = 0.0
    max_angular_velocity = 0.0
    max_angular_accel = 0.0
    points = []

    def __init__(self, max_velocity, max_accel, max_angular_velocity, max_angular_accel, points):
        self.max_velocity = max_velocity
        self.max_accel = max_accel
        self.max_angular_velocity = max_angular_velocity
        self.max_angular_accel = max_angular_accel
        self.points = points

    def build(self):
        return
