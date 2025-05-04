from classes.robot import Robot
from classes.visualize_sim import SimViz
import matplotlib.pyplot as plt
import numpy as np

goal_state = np.array([2., 0., 0.])
    

obst1 = np.array( [ [-1., -1.5], [-0.5, -1.5],\
        [-0.5, -1.], [-1., -1.], [-1, -1.5] ]) 
obst2 = np.array( [ [-1., 1.5], [-0.5, 1.5],\
        [-0.5, 1.], [-1., 1.], [-1, 1.5] ])

obstacles = [obst1, obst2]

robot1_state = np.array([-3.75, 0.5, 0.])
robot2_state = np.array([-3.75, -0.25, 0.])
robot3_state = np.array([-3.25, 0.5, 0.])
robot4_state = np.array([-3.25, -0.25, 0.])

robot1 = Robot(robot_id=1, initial_state=robot1_state, goal_state=goal_state, color='r')
robot2 = Robot(robot_id=2, initial_state=robot2_state, goal_state=goal_state, color='b')
robot3 = Robot(robot_id=3, initial_state=robot3_state, goal_state=goal_state, color='g')
robot4 = Robot(robot_id=4, initial_state=robot4_state, goal_state=goal_state, color='y')

robots = [robot1, robot2, robot3, robot4]

field = ((-4.0, 4.0), (-3.0, 3.0)) # (-x, x), (-y, y)


sim = SimViz(robots=robots, static_obstacles=obstacles, field=field)
sim.init_static_obstacles(obstacles)
sim.run()