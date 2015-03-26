import pyglet
from pyplan.environment import parse_world, SampledExpansionEnvironment
from pyplan.geometry import Polygon, Collection
from pyplan.robot import Point_Robot
from pyplan.renderer import Renderer, KBRLRRT_draw, Enviornment_draw, set_projection
from pyplan.renderer import get_mouse_coord
from pyplan.planner import KBRL_RRT
from pyglet.window import key
import numpy as np

filepath = 'testworld.xml'
robot = Point_Robot()
start, goal, pos_range, polys = parse_world(filepath)
envi = SampledExpansionEnvironment(Collection([Polygon(p) for p in polys]), robot, pos_range)

planner = KBRL_RRT(robot = robot, 
                   environment = envi, 
                   dist_to_goal = 0.5, 
                   max_iterations = 200, 
                   bias = 1, 
                   psi = None, 
                   screenshot_rate = 20, 
                   save_screenshots = True)

path, rrt_data = planner(start, goal)
index_range = [0, len(rrt_data['screenshots'])-1]
index = len(rrt_data['screenshots'])-1

configTemp = pyglet.gl.Config(sample_buffers=1,
    samples=4,
    double_buffer=True,
    alpha_size=0)

platform = pyglet.window.get_platform()
display = platform.get_default_display()
screen = display.get_default_screen()

try:
    config= screen.get_best_config(configTemp)
except:
    config=pyglet.gl.Config(double_buffer=True)

window = pyglet.window.Window(config=config, resizable=True)

render_args = {'num_samples':60, 'log_scale':True}
rrt_renderer = Renderer(data=rrt_data, render_call=KBRLRRT_draw, **render_args)
env_renderer = Renderer(environment=envi, render_call=Enviornment_draw)

@window.event
def on_draw():
    window.clear()
    rrt_renderer.draw()
    env_renderer.draw()
    

@window.event
def on_resize(width, height):
    pyglet.gl.glViewport(0, 0, width, height)
    if envi != None:
        set_projection(envi, width, height)
    else:
        pyglet.gl.glMatrixMode(pyglet.gl.GL_PROJECTION)
        pyglet.gl.glLoadIdentity()
        pyglet.gl.glOrtho(0, 10, 0, 10, -1, 1)
        pyglet.gl.glMatrixMode(pyglet.gl.GL_MODELVIEW)
    return True

@window.event
def on_mouse_scroll(x, y, scroll_x, scroll_y):
    (mx, my)= get_mouse_coord(x, y)
    pyglet.gl.glTranslatef(mx, my, 0)
    pyglet.gl.glScalef(1.05**scroll_y, 1.05**scroll_y, 1)
    pyglet.gl.glTranslatef(-mx, -my, 0)

@window.event
def on_mouse_drag(x, y, dx, dy, buttons, modifiers):
    mcoord1 = get_mouse_coord(x, y)
    mcoord2 = get_mouse_coord(x + dx, y+ dy)
    pyglet.gl.glTranslatef(mcoord2[0] - mcoord1[0], mcoord2[1] - mcoord1[1], 0)

def on_key_press(symbol, modifiers):
    global index, index_range, rrt_renderer
    if symbol == key.RIGHT:
        index = np.clip(index + 1, *index_range)
        rrt_renderer = Renderer(data=rrt_data,
                                render_call=KBRLRRT_draw,
                                index = index, 
                                **render_args)
    if symbol == key.LEFT:
        index = np.clip(index -1, *index_range)
        rrt_renderer = Renderer(data=rrt_data,
                                render_call=KBRLRRT_draw,
                                index = index,
                                **render_args)
        
    if symbol == key.L:
        render_args['log_scale'] = not render_args['log_scale']
        rrt_renderer = Renderer(data=rrt_data,
                                render_call=KBRLRRT_draw,
                                index = index,
                                **render_args)



window.push_handlers(on_key_press)


if __name__ == '__main__':
    pyglet.gl.glEnable(pyglet.gl.GL_BLEND)
    pyglet.gl.glBlendFunc(pyglet.gl.GL_SRC_ALPHA, pyglet.gl.GL_ONE_MINUS_SRC_ALPHA)
    pyglet.gl.glEnable(pyglet.gl.GL_LINE_SMOOTH )
    pyglet.gl.glEnable(pyglet.gl.GL_POLYGON_SMOOTH )
    pyglet.gl.glEnable(pyglet.gl.GL_POINT_SMOOTH )
    pyglet.gl.glClearColor(0, 0, 0, 1.0)
    pyglet.app.run()