import pyglet
from pyplan.environment import parse_world, Environment
from pyplan.geometry import Polygon, Collection
from pyplan.robot import Point_Robot
from pyplan.renderer import Renderer, RRT_draw
from pyplan.planner import RRT

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

filepath = 'test.xml'
robot = Point_Robot()
start, goal, pos_range, polys = parse_world(filepath)
envio = Environment(Collection([Polygon(p) for p in polys]), robot, pos_range)
rrt_planner = RRT(enviornment = envio, robot = robot, step_size = 0.1)

path, rrt_data = rrt_planner(start, goal)
renderer = Renderer(rrt_data, render_call=RRT_draw)

@window.event
def on_draw():
    renderer.draw()