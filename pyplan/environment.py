import numpy as np
from shapely.geometry import MultiPolygon

class Environment(object):

    def __init__(self, obstacles, robot, pos_range, line_check_samples = 3):
        self.obstacles = MultiPolygon(obstacles)
        self.robot = robot
        self.pos_range = pos_range
        self.line_check_samples = line_check_samples

    def check_intersect(self, robot, pos):
        return robot.get_geom(pos).intersects(self.obstacles)

    def check_line_intersect(self, robot, p0, p1):
        return np.any( (self.check_intersect(robot,p)
                        for p in np.linspace(p0, p1, self.line_check_samples)))


