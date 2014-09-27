from shapely.geometry import Polygon

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
      

class Polygon(Geometry):
    """ class for convex polygon intersection test """
    def __init__(self, vertices):
        """ constructor for polygon, vertices must be specified 
        in counter-clockwise order """
        
        # shapely should not be used if the students are implementing this
        self.poly = Polygon(vertices)
        
    def intersects(self, geometry):
        if isinstance(geometry, Collection):
            return geometry.intersects(self)
        else:
            return self.poly_poly_test(geometry)
        
    def poly_poly_test(self, p):
        """ This method should be implemented by the students but for 
        demo purposes, shapely is used """
        return self.poly.intersects(p.poly)
        
        
        
        
