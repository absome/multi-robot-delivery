from classes.robot import Robot
from classes.visualize_sim import SimViz
import matplotlib.pyplot as plt
import numpy as np
import cvxopt
import cvxopt.solvers
def get_obstacle_centers(obstacles):
    """
    Calculate the centroid (geometric center) of a polygon given its vertices.
    
    Parameters:
    vertices (np.ndarray): A numpy array of shape (n, 2) representing the polygon's vertices.
    
    Returns:
    tuple: The (x, y) coordinates of the centroid.
    """
    obstacle_center_points = []
    for vertices in obstacles:
        n = len(vertices)
        sum_area = 0.0
        sum_cx = 0.0
        sum_cy = 0.0

        for i in range(n):
                xi, yi = vertices[i]
                xj, yj = vertices[(i + 1) % n]
                contrib = xi * yj - xj * yi
                sum_area += contrib
                sum_cx += (xi + xj) * contrib
                sum_cy += (yi + yj) * contrib

        area = 0.5 * abs(sum_area)
        if area == 0:
                raise ValueError("The polygon has zero area, centroid is undefined.")

        centroid_x = sum_cx / (6 * area)
        centroid_y = sum_cy / (6 * area)
        obstacle_center_points.append((centroid_x, centroid_y))
    return obstacle_center_points        
        

obst1 = np.array( [ [0., -2.75], [1, -2.75],\
        [1, -1.75], [0., -1.75], [0., -2.75] ]) 
obst2 = np.array( [ [-0.5, 2.0], [0.5, 2.0],\
        [0.5, 1.], [-0.5, 1.], [-0.5, 2.0] ])

obstacles = [obst1, obst2]

# Setup robot initial states and final goal state
goal_state = np.array([4., 0., 0.])
robot1_state = np.array([-3.75, 0.5, 0.])
robot2_state = np.array([-3.75, -0.25, 0.])
robot3_state = np.array([-3.25, 0.5, 0.])
robot4_state = np.array([-3.25, -0.25, 0.])
# Create robot objects
robot1 = Robot(robot_id=1, initial_state=robot1_state, goal_state=goal_state, color='r')
robot2 = Robot(robot_id=2, initial_state=robot2_state, goal_state=goal_state, color='b')
robot3 = Robot(robot_id=3, initial_state=robot3_state, goal_state=goal_state, color='g')
robot4 = Robot(robot_id=4, initial_state=robot4_state, goal_state=goal_state, color='y')
robots = [robot1, robot2, robot3, robot4]

# Define robot pairs and initialize them in each object for QP Control
robo_pairs = [(1,3), (1,4), (1,2), (2,4), (2,3), (4,3)]
obst_centres = [np.array([0., 1.5]), np.array([0.5, -2.25])] # needed for QP
for r in robots:
        r.add_neighbours(robots=robots, pairs=robo_pairs)
        r.add_obstacles(obst_centres) # store obstacle center positions in robot
        
# Setup simulation
field = ((-4.0, 4.0), (-3.0, 3.0)) # (-x, x), (-y, y)

sim = SimViz(robots=robots, static_obstacles=obstacles, field=field)
sim.init_static_obstacles(obstacles)
sim.run()