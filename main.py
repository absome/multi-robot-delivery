from classes.robot import Robot
from classes.visualize_sim import SimViz
import matplotlib.pyplot as plt
import numpy as np



def differential_drive(state, dt):
    # simple forward motion at 0.1 m/s
    x, y, theta = state
    v = 0.1
    return np.array([x + v * np.cos(theta) * dt,
                        y + v * np.sin(theta) * dt,
                        theta])
    

obst1 = np.array( [ [-1., -1.5], [-0.5, -1.5],\
        [-0.5, -1.], [-1., -1.], [-1, -1.5] ]) 
obst2 = np.array( [ [-1., 1.5], [-0.5, 1.5],\
        [-0.5, 1.], [-1., 1.], [-1, 1.5] ])

obstacles = [obst1, obst2]

robot1_state = np.array([-3.75, 0.5, 0.])
robot2_state = np.array([-3.75, -0.25, 0.])
robot3_state = np.array([-3.25, 0.5, 0.])
robot4_state = np.array([-3.25, -0.25, 0.])

robot1 = Robot(robot_id=1, initial_state=robot1_state, color='r', dynamics_func=differential_drive)
robot2 = Robot(robot_id=2, initial_state=robot2_state, color='b', dynamics_func=differential_drive)
robot3 = Robot(robot_id=3, initial_state=robot3_state, color='g', dynamics_func=differential_drive)
robot4 = Robot(robot_id=4, initial_state=robot4_state, color='y', dynamics_func=differential_drive)

robots = [robot1, robot2, robot3, robot4]

field = ((-4.0, 4.0), (-3.0, 3.0)) # (-x, x), (-y, y)


sim = SimViz(robots=robots, static_obstacles=obstacles, field=field)
# sim.draw_static_obstacles(obstacles=obstacles)
# sim.init_icons()
sim.init_static_obstacles(obstacles)
plt.show()