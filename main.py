from classes.robot import Robot
from classes.visualize_sim import SimulationVisualizer
import matplotlib as plt
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

robot1_state = [-2., 1.5, 0.]
robot2_state = [-2., -1., 0.]
robot1 = Robot(robot_id=1, initial_state=robot1_state, color='r', dynamics_func=differential_drive)
robot2 = Robot(robot_id=2, initial_state=robot2_state, color='b', dynamics_func=differential_drive)
robots = [robot1, robot2]

fieldx = (-3.0, 3.0)
fieldy = (-2, 2)
sim = SimulationVisualizer(robots=robots, static_obstacles=obstacles, fieldx=fieldx, fieldy=fieldy)
sim.draw_static_obstacles(obstacles=obstacles)
sim.init_icons()

sim.run()

plt.show()