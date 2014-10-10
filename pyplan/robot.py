import numpy as np
from geometry import Point, transform

class Robot(object):
    def get_dist(self, p0, p1):
        """ This method defines a measure of closeness for some of the
        planners"""
        pass

    def get_geom(self, pos):
        """ This method returns a list of the 2D polygons defined by the robot
            under a given configuration. Returns None if the configuration is
            illegal."""
        pass

    def get_2D_coord(self, pos):
        """ This method returns the 2D position (or the origin) of a given
            configuration on the 2D plane. """
        pass

    def sample_config(self, pos1, pos2, nsamples):
        """ This method returns a series of samples configuration between
            two given configuration points (including the originals) """
        pass


class Point_Robot(Robot):
    def get_dist(self, p0, p1):
        return np.linalg.norm(p0 - p1)

    def get_geom(self, pos):
        return Point(pos)

    def get_2D_coord(self, pos):
        return pos

    def sample_config(self, pos1, pos2, nsamples):
        return np.array([np.linspace(i, j, nsamples) for i,j in zip(pos1, pos2)])

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
        return transform(self.shape, trans)

    def get_2D_coord(self, pos):
        return pos[:2]

    def sample_config(self, pos1, pos2, nsamples):
        return None
