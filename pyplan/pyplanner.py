import argparse
import os
import sys
from pyplan.environment import parse_world, Environment
import pickle

if __name__ == '__main__':
    # parse input arguments if any are given
    parser = argparse.ArgumentParser(description = 'Toy motion planner framework')
    parser.add_argument('--planner',
                        help = 'the path to the planner construction code',
                        default = None)
    parser.add_argument('--robot',
                        help = 'the path to robot construction code',
                        default = None)
    parser.add_argument('--world',
                        help = 'the path to world file',
                        default = None)
    parser.add_argument('--o',
                        help = 'the path to the output file',
                        default = './results.txt')
    parser.add_argument('--f',
                        help = 'allow the output file to be overwritten',
                        action="store_true")
    parser.add_argument('--nogui',
                        help = 'allow the output file to be overwritten',
                        action="store_true")


    args = parser.parse_args()


    planner_filename = args.planner
    robot_filename = args.robot
    world_filename = args.world
    out_filename = args.o
    input_error = False
    
    
    error_msg = '''ERROR: File \'{}\' not found for {}.'''
    overwrite_error_msg = '''ERROR: File \'{}\' exists and will not be 
        overwritten unless the \'--f\' argument is given'''
    code_error_msg= '''ERROR: The {} code must instantiate a variable
        called \'{}\' which implements the {} interface'''
    
    if args.nogui:
        if not os.path.exists(planner_filename):
            input_error = True
            print error_msg.format(planner_filename)
            
        if not os.path.exists(robot_filename):
            input_error = True
            print error_msg.format(robot_filename)
        
        if not os.path.exists(world_filename):
            input_error = True
            print error_msg.format(world_filename)
            
        if os.path.exists(out_filename) and not args.f:
            input_error = True
            print overwrite_error_msg.format( out_filename)
            
        robot = None
        execfile(robot_filename)
        if robot == None:
            input_error = True
            print code_error_msg.format(*(['robot']*3))
            
        start, goal, pos_range, obstacles = parse_world(world_filename)
        enivornment = Environment(obstacles, robot, pos_range)
        
        planner = None
        execfile(planner_filename)
        if planner == None:
            input_error = True
            print code_error_msg.format(*(['planner']*3))
            
        if input_error:
            sys.exit(1)
            
        print 'Running planner...'
        path, data = planner(start, goal)
        with open(out_filename, 'wb') as f:
            pickle.dump(data, f)
            
    else:
        print 'Still no GUI, use the --nogui argument... Work in progress!'
            
       
            