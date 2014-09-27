import numpy as np



def edge_point_test(e1, p1):
    pass

def edge_edge_test(p, q):
    pass
    e1 = p[1] - p[0]
    e2 = q[1] - q[0]
    det = -e1[0]*e2[1] + e1[1]*e2[0]

    # edges are colinear, test for overlap
    if det < 0:
        return edge_point_test(p, q[0]) or edge_point_test(p, q[1])

    x = p[0] - q[0]



class Polygon (object):

    def init(self, vertices, pos):
        self.v = vertices
        self.p = pos

    def contains(self, polygon):
        pass

    def intersects(self, polygon):
        pass

    @property
    def vertices(self):
        return self.v

    @property
    def world_vert(self):
        return self.v + self.p

