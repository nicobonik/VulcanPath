class Pose:
    x = 0.0
    y = 0.0
    heading = 0.0

    def __init__(self, x, y, heading):
        self.x = x
        self.y = y
        self.heading = heading

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_heading(self):
        return self.heading
