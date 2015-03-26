import numpy as np
import xml.etree.ElementTree as ET
from pyplan.geometry import Collection

def sample2DDirection(origin, l, samples):
    theta = np.linspace(0, 2*np.pi, samples)
    x = np.cos(theta)*l
    y = np.sin(theta)*l
    points = np.hstack((x.reshape((-1,1)), y.reshape((-1,1))))
    return points + origin

def parse_world(filename):
    e = ET.parse(filename).getroot()
    polygons = []
    for ce in e.findall('obstacle'):
        vertex = [(float(ve.attrib['x']), float(ve.attrib['y']))
                         for ve in ce.findall('vertex')]
        polygons.append(vertex)

    se = e.find('start')
    start = ( float(se.attrib['x']), float(se.attrib['y']) )

    se = e.find('goal')
    goal = ( float(se.attrib['x']), float(se.attrib['y']) )

    re= e.find('rangex')
    rangex = ( float(re.attrib['min']), float(re.attrib['max']) )

    re= e.find('rangey')
    rangey = ( float(re.attrib['min']), float(re.attrib['max']) )

    return (start,
            goal,
            (np.array([rangex[0], rangey[0]]),
             np.array([rangex[1], rangey[1]])),
            polygons)

class Environment(object):

    def __init__(self, obstacles, robot, config_range, line_check_samples = 3):
        self.obstacles = obstacles
        self.robot = robot
        self.config_range = config_range
        self.line_check_samples = line_check_samples

    def check_intersect(self, robot, pos):
        """ check if a given configuration of the robot violates a constraint
            (e.g., collision with obstacle or impossible configuration. """
        robot_geom = robot.get_geom(pos)
        if robot_geom == None:
            return True
        else:
            return self.obstacles.intersects(robot_geom)

    def check_line_intersect(self, robot, p0, p1):
        geoms = [robot.get_geom(p)
                 for p in robot.sample_config(p0,p1,self.line_check_samples)]
        return self.obstacles.intersects(Collection(geoms))


class SampledExpansionEnvironment(Environment):
    def __init__(self,
                 obstacles, 
                 robot, 
                 config_range,
                 sampler = None,
                 line_check_samples = 3):
        super(SampledExpansionEnvironment, self).__init__(obstacles,
                                                          robot,
                                                          config_range,
                                                          line_check_samples=line_check_samples)
        if sampler is None:
            sampler = lambda x: self.filter_samples(x, sample2DDirection(x, 0.5, 16))
        self.sampler = sampler
        
    def get_best(self, nodes, h_hat):
        sampler = self.sampler
        
        best = np.inf
        
        for v in nodes:
            samples = sampler(v)
            values = h_hat(samples)
            i = values.argmin()
#             print 'values', values
            if values[i] < best:
                argbest = (v, np.linalg.norm(v - samples[i]), samples[i])
                best = values[i]
            
        return argbest
    
    def filter_samples(self, x, samples):
        test = np.zeros(samples.shape[0], dtype='int')
        for i,s in enumerate(samples):
            test[i] = not self.check_line_intersect(self.robot, x, s)
            
        return samples[test.nonzero()]
            
            
            
        
        