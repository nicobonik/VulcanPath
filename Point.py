class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def getX(self):
        return self.x

    def getY(self):
        return self.y

    def setPoint(self, point):
        self.x = point.x
        self.y = point.y


class PathPoint:
    def __init__(self, x, y, speed, lookahead):
        self.x = x
        self.y = y
        self.speed = speed
        self.lookahead = lookahead

    def set_pathpoint(self, pathpoint):
        self.x = pathpoint.x
        self.y = pathpoint.y
        self.speed = pathpoint.speed
        self.lookahead = pathpoint.lookahead

    def set_point(self, point):
        self.x = point.x
        self.y = point.y

    def to_point(self):
        return Point(self.x, self.y)
