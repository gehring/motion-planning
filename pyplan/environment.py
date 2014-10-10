import numpy as np
import xml.etree.ElementTree as ET
from pyplan.geometry import Collection

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
        return False#self.obstacles.intersects(Collection(geoms))


