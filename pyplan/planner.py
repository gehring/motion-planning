import numpy as np

class UniformSampler(object):
    def __init__(self, sample_range):
        self.range = sample_range

    def __call__(self):
        x = np.random.uniform(self.range[0].size)
        return (x-self.range[0])/(self.range[1]-self.range[0])


class Planner(object):
    def __call__(self, start, goal):
        """ find a path from start to goal. Return a tuple of the path
        (None if none is found) with a dict of extra data (for rendering
        the results).
        E.g., return path, {'start':tuple(start),
                      'goal':tuple(goal),
                      'path':path,
                      'screenshots':screenshots}"""
        pass

class RRT(Planner):
    def __init__(self,
                 robot,
                 environment,
                 step_size,
                 sampler = None,
                 max_iterations=5000,
                 screenshot_rate = 1,
                 save_screenshots = False):
        self.sampler = sampler
        self.environment = environment
        self.robot = robot
        self.step_size = step_size
        self.max_iterations = max_iterations
        self.screenshot_rate = 1,
        self.save_screenshots = save_screenshots

    def __call__(self,
                 start,
                 goal):
        sampler = self.sampler
        robot = self.robot
        environment = self.environment
        if sampler == None:
            sampler = UniformSampler(environment.config_range)

        tree = {tuple(start):None}
        last_point = start
        count = 0
        screenshots = []
        while robot.get_dist(np.array(last_point), np.array(goal)) > self.step_size \
                    and count < self.max_iterations:
            if self.save_screenshots and count % self.screenshot_rate == 0:
                screenshots.append(tree.copy())
            last_point, near = self.sample_new_point(sampler,
                                                     tree,
                                                     self.step_size,
                                                     robot,
                                                     environment)
            tree[tuple(last_point)] = near
            count += 1

        if robot.get_dist( np.array(last_point),  np.array(goal)) < self.step_size:
            tree[tuple(goal)] = tuple(last_point)
            path = self.get_path(tuple(goal), tree)
        else:
            path = None
        screenshots.append(tree.copy())

        return path, {'start':tuple(start),
                      'goal':tuple(goal),
                      'path':path,
                      'robot':robot,
                      'screenshots':screenshots}



    def sample_new_point(self, sampler, tree, step_size, robot, environment):
        point = sampler()
        near = min(tree.iterkeys(), key= lambda x: np.linalg.norm(point - x))
        d = robot.get_dist( np.array(point),  np.array(near))
        point -= near
        point *= step_size/d

        while environment.check_line_intersect(robot, near, point):
            point = sampler()
            near = min(tree.iterkeys(), key= lambda x: np.linalg.norm(point - x))
            d = robot.get_dist( np.array(point),  np.array(near))
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


