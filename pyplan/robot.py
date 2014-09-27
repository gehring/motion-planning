import numpy as np
from shapely.geometry import Point
from shapely.affinity import affine_transform

class Robot(object):
    def get_dist(self, p0, p1):
        pass

    def get_geom(self, pos):
        pass

    def get_norm(self, pos):
        pass

class Point_Robot(Robot):
    def get_dist(self, p0, p1):
        return np.linalg.norm(p0-p1)

    def get_geom(self, pos):
        return Point(pos)

    def get_norm(self, pos):
        return np.linalg.norm(pos)

class Poly_Robot(Robot):
    angle_ratio = 0.1

    def __init__(self, shape):
        self.shape

    def get_dist(self, p0, p1):
        d = np.linalg.norm(p0[:-1]-p1[:-1])
        a = p0[-1] - p1[-1]
        if a < 0:
            a = min(-a, a+np.pi*2)
        else:
            a = min(a, np.pi*2-a)
        return d + self.angle_ratio * a * a

    def get_geom(self, pos):
        angle = pos[-1]
        s = np.sin(angle)
        c = np.cos(angle)
        trans = [c, -s, s, c, pos[0], pos[1]]
        return affine_transform(self.shape, trans)

    def get_norm(self, pos):
        d = np.linalg.norm(pos[:-1])
        a = pos[-1] - np.pi*2
        if a < 0:
            a = min(-a, a+np.pi*2)
        else:
            a = min(a, np.pi*2-a)
        return d + self.angle_ratio * a * a