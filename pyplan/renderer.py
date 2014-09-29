import pyglet
from itertools import chain

RRT_RENDER_CACHE = {}


class line_point_group(pyglet.graphics.Group):
    def __init__(self, line_width, point_width,  parent = None):
        super(line_point_group, self).__init__(parent=parent)
        self.line_width = line_width
        self.point_width = point_width

    def set_state(self):
        pyglet.gl.glLineWidth(self.line_width)
        pyglet.gl.glPointSize(self.point_width)

    def unset_state(self):
        pass

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
    """ Render a specific screenshot of a RRT algorithm. This method will
        cache (with strong references) the most recent input so memory of the
        cache arguments won't be released immediately. """

    global RRT_RENDER_CACHE
    cache_key = (rrt_data,
                 index,
                 goal_color,
                 start_color,
                 node_color,
                 edge_color,
                 path_node_color,
                 path_edge_color,
                 edge_width,
                 node_width,
                 path_edge_width,
                 path_node_width,
                 start_goal_width)

    if not cache_key in RRT_RENDER_CACHE:
        batch = pyglet.graphics.Batch()
        tree = rrt_data['screenshots'][index]
        path = rrt_data['path']
        robot = rrt_data['robot']


        # set up rendering properties -------------------------

        # these groups enforce the rendering order
        tree_order = pyglet.graphics.OrderedGroup(0)
        path_order = pyglet.graphics.OrderedGroup(1)
        goal_order = pyglet.graphics.OrderedGroup(2)

        # these groups inherent the render order from their parents
        # and set the line and point widths to use during render
        tree_group = line_point_group(line_width = edge_width,
                                      point_width = node_width,
                                      parent = tree_order)
        path_group = line_point_group(line_width = path_edge_width,
                                      point_width = path_node_width,
                                      parent = path_order)
        goal_group = line_point_group(line_width = start_goal_width,
                                      point_width = start_goal_width,
                                      parent = goal_order)

        # set up the required rendering calls -----------------

        if tree != None:
            # draw edges of the tree
            edges = [robot.get_2D_coord(e) for e in tree.iteritems()
                            if e[0] != rrt_data['start'] for p in e]
            batch.add(len(edges)/2, pyglet.gl.GL_LINES, tree_group
                                     ('v2f', edges),
                                     ('c4B', edge_color*(len(edges)/2)))

            # draw nodes of the tree
            nodes = [robot.get_2D_coord(v) for v in tree.keys()]
            batch.add(len(nodes), pyglet.gl.GL_POINTS, tree_group
                                     ('v2f', nodes),
                                     ('c4B', node_color*(len(nodes))))
        if path != None:
            # draw edges of the path
            index = ( (i,i) for i in xrange(1, len(path)-1))
            edges = [robot.get_2D_coord(path[i]) for i in chain((0), *index)]
            batch.add(len(edges)/2, pyglet.gl.GL_LINES, path_group
                                     ('v2f', edges),
                                     ('c4B', edge_color*(len(edges)/2)))

            # draw nodes of the path
            nodes = [ robot.get_2D_coord(v) for v in path]
            batch.add(len(nodes), pyglet.gl.GL_POINTS, path_group
                                     ('v2f', nodes),
                                     ('c4B', node_color*(len(nodes))))





        # draw start and goal
        nodes = (robot.get_2D_coord(rrt_data['start']),
                 robot.get_2D_coord(rrt_data['goal']))
        color = (start_color, goal_color)
        batch.add(2, pyglet.gl.GL_POINTS, goal_group,
                                 ('v2f', nodes),
                                 ('c4B', color))

        RRT_RENDER_CACHE = {cache_key: batch}
    RRT_RENDER_CACHE[cache_key].draw()
