from shapely.geometry import Polygon as ShapelyPolygon
from shapely.geometry import Point as ShapelyPoint
from shapely.affinity import affine_transform

class Geometry(object):
    def intersects(self, geometry):
        pass

class Collection(Geometry):
    """ class for a collection of intersectables """
    def __init__(self, geometries):
        """ constructor for the collection """
        self.geoms = geometries

    def intersects(self, geometry):
        test = False
        for g in self.geoms:
            test |= geometry.intersects(g)
            if test:
                break
        return test

class Point(Geometry):
    """ class for a point """
    def __init__(self, point):
        # shapely should not be used if the students are implementing this
        self.point = ShapelyPoint(point)

    def intersects(self, geometry):
        if isinstance(geometry, Point):
            return False
        else:
            return geometry.intersects(self)

    @property
    def coord(self):
        return (self.point.x, self.point.y)

class Polygon(Geometry):
    """ class for convex polygon intersection test """
    def __init__(self, vertices):
        """ constructor for polygon, vertices must be specified
        in counter-clockwise order """

        # shapely should not be used if the students are implementing this
        self.poly = ShapelyPolygon(vertices)

    def intersects(self, geometry):
        if isinstance(geometry, Collection):
            return geometry.intersects(self)
        elif isinstance(geometry, Point):
            return self.point_poly_test(geometry)
        else:
            return self.poly_poly_test(geometry)

    def point_poly_test(self, p):
        """ This method should be implemented by the students but for
        demo purposes, shapely is used """
        return self.poly.intersects(p.point)

    def poly_poly_test(self, p):
        """ This method should be implemented by the students but for
        demo purposes, shapely is used """
        return self.poly.intersects(p.poly)

    @property
    def vertices(self):
        return list(self.poly.exterior.coords)


def transform(shape, trans):
    if isinstance(shape, Collection):
        return Collection([transform(s, trans) for s in shape.geoms])
    elif isinstance(shape, Polygon):
        # shapely should not be used if the students are implementing this
        return Polygon(affine_transform(shape.poly, trans).exterior.coord)
    else:
        # shapely should not be used if the students are implementing this
        return Point(affine_transform(shape.point, trans).coords[0])
