import pyglet
from itertools import chain
from geometry import Collection, Polygon

class line_point_group(pyglet.graphics.Group):
    def __init__(self, line_width, point_width,  parent = None):
        super(line_point_group, self).__init__(parent=parent)
        self.line_width = line_width
        self.point_width = point_width

    def set_state(self):
        pyglet.gl.glLineWidth(self.line_width)
        pyglet.gl.glPointSize(self.point_width)


class Renderer(object):
    def __init__(self, **kargs):
        render_call = kargs['render_call']
        del kargs['render_call']
        self.batch = render_call(**kargs)

    def draw(self):
        self.batch.draw()

def RRT_draw(rrt_data,
             index = -1,
             goal_color = (200, 200, 100, 255),
             start_color = (200, 150, 200, 255),
             node_color = (150, 150, 100, 255),
             edge_color = (200, 150, 100, 255),
             path_node_color = (150, 150, 150, 255),
             path_edge_color = (100, 150, 100, 255),
             edge_width = 1.0,
             node_width = 3.0,
             path_edge_width = 2.0,
             path_node_width = 6.0,
             start_goal_width = 8.0):
    """ Get a batch renderer for a specific screenshot of a RRT algorithm. """


    batch = pyglet.graphics.Batch()
    tree = rrt_data['screenshots'][index]
    path = rrt_data['path']
    robot = rrt_data['robot']


    # set up rendering properties -------------------------

    # these groups enforce the rendering order
    tree_order = pyglet.graphics.OrderedGroup(0)
    path_order = pyglet.graphics.OrderedGroup(1)
    goal_order = pyglet.graphics.OrderedGroup(2)

    # these groups inherit the render order from their parents
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
        edges = [x for e in tree.iteritems()
                        if e[0] != rrt_data['start'] for p in e
                                                for x in robot.get_2D_coord(p)]
        batch.add(len(edges)/2, pyglet.gl.GL_LINES, tree_group,
                                 ('v2f', edges),
                                 ('c4B', edge_color*(len(edges)/2)))

        # draw nodes of the tree
        nodes = [ x for v in tree.keys()
                            for x in robot.get_2D_coord(v)]
        batch.add(len(nodes)/2, pyglet.gl.GL_POINTS, tree_group,
                                 ('v2f', nodes),
                                 ('c4B', node_color*(len(nodes)/2)))
    if path != None:
        # draw edges of the path
        index = [(i,i) for i in xrange(1, len(path)-1)] + [[len(path)-1]]
        edges = [x for i in chain([0], *index)
                                        for x in robot.get_2D_coord(path[i])]
        batch.add(len(edges)/2, pyglet.gl.GL_LINES, path_group,
                                 ('v2f', edges),
                                 ('c4B', edge_color*(len(edges)/2)))

        # draw nodes of the path
        nodes = [ x for v in path for x in robot.get_2D_coord(v)]
        batch.add(len(nodes)/2, pyglet.gl.GL_POINTS, path_group,
                                 ('v2f', nodes),
                                 ('c4B', node_color*(len(nodes)/2)))





    # draw start and goal
    nodes = (robot.get_2D_coord(rrt_data['start']),
             robot.get_2D_coord(rrt_data['goal']))
    nodes = tuple(chain(*nodes))
    color = tuple(chain(start_color, goal_color))
    batch.add(2, pyglet.gl.GL_POINTS, goal_group,
                             ('v2f', nodes),
                             ('c4B', color))

    return batch

def Enviornment_draw(environment,
                     obs_color = (200, 200, 200, 255),
                     edge_width = 2.0,
                     point_width = 3.0):

    batch = pyglet.graphics.Batch()
    obs =  environment.obstacles
    group = line_point_group(line_width = edge_width,
                                  point_width = point_width)
    add_polygon_render(obs, group, batch, obs_color)
    return batch

def add_polygon_render(poly, group, batch, color):
    if isinstance(poly, Collection):
        for g in poly.geoms:
            add_polygon_render(g, group, batch, color)
    elif isinstance(poly, Polygon):
        vertices = poly.vertices
        index = [ (i,i) for i in xrange(1, len(vertices))] + [[0]]
        edges = [x for i in chain([0], *index) for x in vertices[i]]
        batch.add(len(edges)/2, pyglet.gl.GL_LINES, group,
                                     ('v2f', edges),
                                     ('c4B', color*(len(edges)/2)))
    else:
        batch.add(1, pyglet.gl.GL_POINTS, group,
                    ('v2f', poly.coord),
                    ('c4B', color))

def set_projection(environment, width, height):
        pyglet.gl.glMatrixMode(pyglet.gl.GL_PROJECTION)
        pyglet.gl.glLoadIdentity()

        rangex = (environment.config_range[0][0], environment.config_range[1][0])
        rangey = (environment.config_range[0][1], environment.config_range[1][1])

        ratio = float(height)/width
        lx = rangex[1] - rangex[0]
        ly = rangey[1] - rangey[0]

        if lx*ratio >= ly:
            dy = lx*ratio - ly
            pyglet.gl.glOrtho(rangex[0], rangex[1], rangey[0]- dy/2, rangey[1]+dy/2, -1, 1)
        else:
            dx = ly/ratio - lx
            pyglet.gl.glOrtho(rangex[0]-dx/2, rangex[1] + dx/2, rangey[0], rangey[1], -1, 1)


        pyglet.gl.glMatrixMode(pyglet.gl.GL_MODELVIEW)

def get_mouse_coord(x, y):
        vp = (pyglet.gl.GLint * 4)()
        mvm = (pyglet.gl.GLdouble * 16)()
        pm = (pyglet.gl.GLdouble * 16)()

        pyglet.gl.glGetIntegerv(pyglet.gl.GL_VIEWPORT, vp)
        pyglet.gl.glGetDoublev(pyglet.gl.GL_MODELVIEW_MATRIX, mvm)
        pyglet.gl.glGetDoublev(pyglet.gl.GL_PROJECTION_MATRIX, pm)

        wx = pyglet.gl.GLdouble()
        wy = pyglet.gl.GLdouble()
        wz = pyglet.gl.GLdouble()

        pyglet.gl.gluUnProject(x, y, 0, mvm, pm, vp, wx, wy, wz)
        mcoord = (wx.value, wy.value)

        return mcoord

