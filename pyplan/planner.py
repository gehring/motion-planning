import numpy as np
import imp

class UniformSampler(object):
    def __init__(self, sample_range):
        self.range = sample_range

    def __call__(self):
        x = np.random.rand(self.range[0].size)
        return x*(self.range[1]-self.range[0]) + self.range[0]


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
        self.screenshot_rate = screenshot_rate
        self.save_screenshots = save_screenshots

    def __call__(self,
                 start,
                 goal):
        sampler = self.sampler
        robot = self.robot
        environment = self.environment
        if sampler is None:
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
            print count

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
        near = min(tree.iterkeys(), key= lambda x: robot.get_dist(np.array(point),  np.array(x)))
        d = robot.get_dist( np.array(point),  np.array(near))
        point -= near
        if d> step_size:
            point *= step_size/d

        while environment.check_line_intersect(robot, np.array(near), point+near):
            point = sampler()
            near = min(tree.iterkeys(), key= lambda x: robot.get_dist(np.array(point),  np.array(x)))
            d = robot.get_dist( np.array(point),  np.array(near))
            point -= near
            if d> step_size:
                point *= step_size/d
        return point+near, near

    def get_path(self, point, tree):
        path = [point]
        while tree[point] != None:
            point = tree[point]
            path.append(point)
        path.reverse()
        return path
    
def generate_path(self, parents, point):
        path = [point]
        
        # go up the parent pointer tree to the root
        while tuple(point) in parents:
            point = parents[tuple(point)]
            path.append(point)
        path.reverse()
        return path

try:
    imp.find_module('rltools')
    found = True
except ImportError:
    found = False
    
if found:
    from rltools.kbrlrrt import KBRLRRT as RL_planner
    from rltools.kbrlrrt import RBF_Kernel, \
                                approx_cost_to_go, \
                                solve_values_plus_cost

    class KBRL_RRT(Planner):
        def __init__(self,
                     robot,
                     environment,
                     sampler = None,
                     dist_to_goal = 1.0,
                     max_iterations=5000,
                     bias = 1.0,
                     psi = None,
                     screenshot_rate = 1,
                     save_screenshots = False):
            
            heuristic = lambda s, t: robot.get_dist(s, t)
            env = environment
            if sampler is None:
                sampler = UniformSampler(environment.config_range)
            if psi is None:
                psi = RBF_Kernel(np.ones(len(environment.config_range))*1.0)
                
            self.dist_to_goal = dist_to_goal
            self.rl_planner = RL_planner(env, heuristic, sampler, bias, psi)
            self.robot = robot
            self.max_iterations = max_iterations
            self.screenshot_rate = screenshot_rate
            self.save_screenshots = save_screenshots
            
        def __call__(self, start, goal):
            start = np.array(start)
            goal = np.array(goal)
            
            goal_test = lambda x: self.robot.get_dist(x, goal) < self.dist_to_goal
            path, data = self.rl_planner.plan(start, 
                                              goal, 
                                              goal_test,
                                              screenshot_rate = self.screenshot_rate,
                                              save_screenshot = self.save_screenshots, 
                                              max_iterations = self.max_iterations)
            data['robot'] = self.robot
            return path, data
        
    class KBRL_RRTV2(Planner):
        def __init__(self,
                     robot,
                     environment,
                     sampler = None,
                     dist_to_goal = 1.0,
                     max_iterations=5000,
                     bias = 1.0,
                     psi = None,
                     screenshot_rate = 1,
                     save_screenshots = False):
            
            self.heuristic = lambda s, t: robot.get_dist(s, t)
            self.env = environment
            if sampler is None:
                sampler = UniformSampler(environment.config_range)
            if psi is None:
                psi = RBF_Kernel(np.ones(len(environment.config_range))*1.0)
               
            self.bias = bias 
            self.sampler
            self.psi = psi
            self.dist_to_goal = dist_to_goal
            self.robot = robot
            self.max_iterations = max_iterations
            self.screenshot_rate = screenshot_rate
            self.save_screenshots = save_screenshots
            
        def __call__(self, start, goal):
            env = self.env
            heuristic = self.heuristic
            sampler = self.sampler
            save_screenshot = self.save_screenshots
            screenshot_rate = self.screenshot_rate
            
            start = np.array(start)
            goal = np.array(goal)
            
            goal_test = lambda x: self.robot.get_dist(x, goal) < self.dist_to_goal
            
            # sample new point to boot strap the process
            point = sampler()
            h_hat = lambda x: heuristic(x, point)
    #         h_hat = lambda x: heuristic(x, goal)
            origin, cost, next_point = env.get_best([start], h_hat)
            
            # initialize the sample set
            samples = (np.array([start], dtype='float32'),
                       np.array([cost], dtype='float32'),
                       np.array([next_point], dtype='float32'))
            
            # initialize the parent pointer tree structure
            parents = {tuple(next_point):tuple(start)}
            
            # all nodes
            nodes = [start, next_point]
            
            # set of nodes with children
            has_child = set()
            has_child.add(tuple(start))
            count = 0
            failed = False
            screenshots = []
            
            while not goal_test(next_point):
                # sample random destination point
                point = sampler()
    
                
    
                # if asked, take a screenshot of the state
                if save_screenshot and (count % screenshot_rate) == 0:
                    vpc = solve_values_plus_cost(samples, 
                                                 goal, 
                                                 self.psi, 
                                                 self.bias, 
                                                 heuristic)
                    h_hat = approx_cost_to_go(goal, 
                                         self.psi, 
                                         vpc.copy(), 
                                         self.bias, 
                                         heuristic, 
                                         [samples[i].copy() for i in range(3)])   
                    screenshots.append( (parents.copy(), h_hat))
    
    
                # approximate cost-to-go heuristic
                vpc = solve_values_plus_cost(samples, 
                                                 goal, 
                                                 self.psi, 
                                                 self.bias, 
                                                 heuristic)
                # update heuristic
                h_hat = approx_cost_to_go(point, 
                                         self.psi, 
                                         vpc, 
                                         self.bias, 
                                         heuristic, 
                                         samples)   
                
                
                # find the best expansion
                origin, cost, next_point = env.get_best(nodes, h_hat)
                
                
                # if origin is not in the samples, add it
                if tuple(origin) not in has_child:
    #                 samples = ( np.vstack((samples[0], [origin])),
    #                             np.hstack((samples[1], [cost])),
    #                             np.vstack((samples[2], [next_point])))
                    has_child.add(tuple(origin))
                    
                samples = ( np.vstack((samples[0], [origin])),
                                np.hstack((samples[1], [cost])),
                                np.vstack((samples[2], [next_point])))
                
                # add new point to the parent pointer tree
                parents[tuple(next_point)] = tuple(origin)
                nodes.append(next_point)
                count += 1
                print count
                
                
                if count >= self.max_iterations:
                    failed = True
                    break
            
            # extract the path from the parent pointer tree
            if not failed:
                path = generate_path(parents, next_point)
                path.append(goal)
            else:
                path = None
                
            
            # process the last screenshot
            vpc = solve_values_plus_cost(samples, 
                                                 goal, 
                                                 self.psi, 
                                                 self.bias, 
                                                 heuristic)
            h_hat = approx_cost_to_go(goal, 
                                         self.psi, 
                                         vpc.copy(), 
                                         self.bias, 
                                         heuristic, 
                                         [samples[i].copy() for i in range(3)])    
            screenshots.append((parents, h_hat))
            
            
            return path, {'start':tuple(start),
                          'goal': tuple(goal),
                          'path':path,
                          'environment': env,
                          'screenshots': screenshots}
            
        def sample_new_point(self, sampler, h_hat, nodes, samples, step_size, robot, environment):
            point = sampler()
            
            dp = point [None,:] - samples
            
            
            
            values = h_hat(points)
            i = values.argmin()
            near = points[i]

            d = robot.get_dist( np.array(point),  np.array(near))
            point -= near
            if d> step_size:
                point *= step_size/d
    
            while environment.check_line_intersect(robot, np.array(near), point+near):
                point = sampler()
                near = min(tree.iterkeys(), key= lambda x: robot.get_dist(np.array(point),  np.array(x)))
                d = robot.get_dist( np.array(point),  np.array(near))
                point -= near
                if d> step_size:
                    point *= step_size/d
            return point+near, near
