import numpy as np
import pyglet

class UniformSampler(object):
    def __init__(self, sample_range):
        self.range = sample_range

    def __call__(self):
        x = np.random.uniform(self.range[0].size)
        return (x-self.range[0])(self.range[1]-self.range[0])


class RRT(object):
    def __init__(self, robot, enivornment, sampler = None):
        self.sampler = sampler
        self.environment = enivornment
        self.robot = robot

    def __call__(self,
                 start,
                 goal,
                 step_size,
                 max_iterations=5000,
                 screenshot_rate = 1,
                 save_screenshots = False):
        sampler = self.sampler
        robot = self.robot
        environment = self.environment
        if sampler == None:
            sampler = UniformSampler(environment.pos_range)

        tree = {tuple(start):None}
        last_point = start
        count = 0
        screenshots = []
        while robot.get_dist(last_point, goal) > step_size \
                    and count < max_iterations:
            if save_screenshots and count % screenshot_rate == 0:
                screenshots.append(tree.copy())
            last_point, near = self.sample_new_point(sampler,
                                                     tree,
                                                     step_size,
                                                     robot,
                                                     environment)
            tree[tuple(last_point)] = near
            count += 1

        if robot.get_dist(last_point, goal) > step_size:
            tree[tuple(goal)] = last_point
            path = self.get_path(goal, tree)
        else:
            path = None
        screenshots.append(tree.copy())

        return path, {'start':tuple(start),
                      'goal':tuple(goal),
                      'path':path,
                      'screenshots':screenshots}



    def sample_new_point(self, sampler, tree, step_size, robot, environment):
        point = sampler()
        near = min(tree.iterkey(), key= lambda x: np.linalg.norm(point - x))
        d = robot.get_dist(point, near)
        point -= near
        point *= step_size/d

        while environment.check_line_intersect(robot, near, point):
            point = sampler()
            near = min(tree.iterkey(), key= lambda x: np.linalg.norm(point - x))
            d = robot.get_dist(point, near)
            point -= near
            point *= step_size/d
        return point, near

    def get_path(self, point, tree):
        path = [point]
        while tree[point] != None:
            point = tree[point]
            path.append(point)
        path.reverse()
        return path

def RRT_draw(self,
             rrt_data,
             index = -1,
             goal_color = (200, 150, 100, 255),
             start_color = (200, 150, 100, 255),
             node_color = (200, 150, 100, 255),
             edge_color = (200, 150, 100, 255),
             path_node_color = (200, 150, 100, 255),
             path_edge_color = (200, 150, 100, 255),
             edge_width = 1.0,
             node_width = 1.0,
             path_edge_width = 2.0,
             path_node_width = 2.0,
             start_goal_width = 3.0):

    tree = rrt_data['screenshots'][index]

    # draw edges of the tree
    edges = [p for e in tree.iteritems() for p in e]
    pyglet.gl.glLineWidth(edge_width)
    pyglet.graphics.draw(len(edges)/2, pyglet.gl.GL_LINES,
                             ('v2f', edges),
                             ('c4B', edge_color*(len(edges)/2)))

    # draw nodes of the tree
    nodes = tree.keys()
    pyglet.gl.glPointSize(node_width)
    pyglet.graphics.draw(len(nodes), pyglet.gl.GL_POINTS,
                             ('v2f', nodes),
                             ('c4B', node_color*(len(nodes))))





    # draw start and goal
    nodes = (rrt_data['start'], rrt_data['goal'])
    color = (start_color, goal_color)
    pyglet.gl.glPointSize(start_goal_width)
    pyglet.graphics.draw(2, pyglet.gl.GL_POINTS,
                             ('v2f', nodes),
                             ('c4B', color))

