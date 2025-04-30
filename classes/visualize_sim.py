from robot import Robot

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class SimulationVisualizer:
    """
    Handles visualization for a multi-robot system
    """
    def __init__(self, robots, static_obstacles, fieldx, fieldy):
        # FPS, interval ms between frames and number of frames
        interval = NotImplemented
        n_frames = NotImplemented
        
        # Plot parameters
        self.fig = plt.figure(1)
        self.ax = plt.gca()
        self.ax.set(xlabel="x [m]", ylabel="y [m]")
        xlim, ylim = fieldx, fieldy
        print(xlim)
        self.set_field(x_axis_range=xlim, y_axis_range=ylim)
        self.ax.set_aspect('equal', adjustable='box', anchor='C')
        plt.tight_layout()
        
        # Set text to count simulationt time
        self.time_txt = self.ax.text(0.78, 0.01, 't = 0 s', color = 'k', fontsize='large', 
            horizontalalignment='left', verticalalignment='bottom', transform = self.ax.transAxes)
        
        # Store obstacle icon and trajectory for each robot id
        self.robot_visuals = {}
        for robot in robots:
            self.robot_visuals[robot.id] = {
                "line": None,
                "icon": []
            }

        self.static_obstacles = static_obstacles
        self.moving_obstacles = []
        self.robots = robots
        
    def init_icons(self):
        for robot in self.robots:
            px = robot.state[0]
            py = robot.state[1]
            theta = robot.state[2]

            # Robot size params
            scale = 2
            body_radius = 0.08 * scale
            scale = 2
            body_rad = 0.08 * scale # m
            wheel_length = 0.1*scale
            wheel_width = 0.02*scale
            arrow_size = body_rad
            
            # Robot body
            body = plt.Circle( (px, py), body_rad, color=robot.color)

            # Wheels
            wheel_angle = [theta + 0., theta+np.pi]
            wheel_angle_degrees = [np.rad2deg(i) for i in wheel_angle]
            wh_x = [ px - body_rad*np.sin(i) - (wheel_length/2)*np.cos(i) + (wheel_width/2)*np.sin(i) for i in wheel_angle ]
            wh_y = [ py + body_rad*np.cos(i) - (wheel_length/2)*np.sin(i) - (wheel_width/2)*np.cos(i) for i in wheel_angle ]
            
            w1 = plt.Rectangle( (wh_x[0], wh_y[0]), wheel_length, wheel_width,
                               angle=wheel_angle_degrees[0], color='k')
            w2 = plt.Rectangle( (wh_x[1], wh_y[1]), wheel_length, wheel_width,
                               angle=wheel_angle_degrees[1], color='k')
                
            # Arrow
            ar_st= [px, py] #[ px - (arrow_size/2)*np.cos(th), py - (arrow_size/2)*np.sin(th) ]
            ar_d = (arrow_size*np.cos(theta), arrow_size*np.sin(theta))

        
            line, = self.ax.plot([], [], linestyle='--', color=robot.color)

            # Add patch
            self.ax.add_patch(body)
            self.ax.add_patch(w1)
            self.ax.add_patch(w2)
            # arrow drawn after for visibility
            arrow = plt.quiver( ar_st[0], ar_st[1], ar_d[0], ar_d[1], 
                scale_units='xy', scale=1, color='k', width=0.1*arrow_size)
            
            self.robot_visuals[robot.id]["line"] = line
            self.robot_visuals[robot.id]["icon"] = [body, arrow, w1, w2]
            
            # body_radius = 0.1
            # circ = plt.Circle((0, 0), body_radius, color=robot.color, alpha=0.5)
            # self.ax.add_patch(circ)

            # # heading arrow
            # arr = self.ax.quiver(0, 0, .5, 0, scale_units='xy', scale=1, color='k', width=0.002)

            # # wheels: two small rectangles perpendicular to heading
            # wheel_length = 0.12
            # wheel_width = 0.03
            # # placeholders: actual positions set in init_draw
            # w1 = plt.Rectangle((0, 0), wheel_length, wheel_width, angle=0, color='k')
            # w2 = plt.Rectangle((0, 0), wheel_length, wheel_width, angle=0, color='k')
            # self.ax.add_patch(w1)
            # self.ax.add_patch(w2)
            
            # self.robot_visuals[robot.id]["line"] = line
            # self.robot_visuals[robot.id]["icon"] = [circ, arr, w1, w2] 
    
    # def init_draw(self):
    #     """
    #     For each robot, stores its visual characteristics into an array
    #     and returns that array
    #     """
    #     # Store robot visuals for each robot
    #     artists = []
        
    #     for line, circle, arrow, robot in zip(self.trajectory_lines,
    #                                       self.patch_circle,
    #                                       self.patch_arrow,
    #                                       self.robots):
    #         line.set_data([], [])
    #         x, y, theta = robot.state
    #         circle_center = (x, y)
    #         arrow_x, arrow_y = np.cos(theta), np.sin(theta)
    #         artists.extend([line, circle, arrow])
        
    #     return artists
    
    # def update(self):
    #     raise NotImplementedError

    def set_field(self, x_axis_range, y_axis_range):
        """
        Sets the range for each axis, x and y on the graph
        """
        self.ax.set( xlim=x_axis_range, ylim = y_axis_range)
        
    def show_goal(self):
        raise NotImplementedError
    
    def draw_robot(self, robot_state):
        raise NotImplementedError
    
    def run(self):
        raise NotImplementedError
    
    def draw_static_obstacles(self, obstacles):
        for obstacle in obstacles:
            self.ax.plot( obstacle[:,0], obstacle[:,1], '--r' )


########## TEMPLATE ###################
# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation


# class Simulation:
#     """
#     Manages multiple robots and animates their trajectories using FuncAnimation.
#     Each robot is drawn as a filled circle with two side wheels and a heading arrow.
#     """
#     def __init__(self, robots, field=((-2, 2), (-2, 2)), interval=100, n_frames=200):
#         self.robots = robots
#         self.interval = interval  # ms between frames
#         self.n_frames = n_frames

#         # Setup figure and axes
#         self.fig, self.ax = plt.subplots()
#         self.ax.set(xlabel='x [m]', ylabel='y [m]', aspect='equal')
#         xlim, ylim = field
#         self.ax.set(xlim=xlim, ylim=ylim)

#         # Prepare artists containers
#         self.traj_lines = []      # Line2D for each robot trajectory
#         self.icon_patches = []    # list of lists: [circle, arrow, wheel1, wheel2]

#         # Initialize artists for each robot
#         for robot in self.robots:
#             # trajectory line
#             line, = self.ax.plot([], [], linestyle='--', color=robot.color)

#             # draw body circle (filled)
#             body_radius = 0.1
#             circ = plt.Circle((0, 0), body_radius, color=robot.color, alpha=0.5)
#             self.ax.add_patch(circ)

#             # heading arrow
#             arr = self.ax.quiver(0, 0, 1, 0, scale_units='xy', scale=1, color='k', width=0.02)

#             # wheels: two small rectangles perpendicular to heading
#             wheel_length = 0.12
#             wheel_width = 0.03
#             # placeholders: actual positions set in init_draw
#             w1 = plt.Rectangle((0, 0), wheel_length, wheel_width, angle=0, color='k')
#             w2 = plt.Rectangle((0, 0), wheel_length, wheel_width, angle=0, color='k')
#             self.ax.add_patch(w1)
#             self.ax.add_patch(w2)

#             self.traj_lines.append(line)
#             self.icon_patches.append([circ, arr, w1, w2])

#     def init_draw(self):
#         """
#         Initialize all artists to their starting positions.
#         Called once by FuncAnimation.
#         """
#         artists = []
#         for line, patches, robot in zip(self.traj_lines, self.icon_patches, self.robots):
#             circ, arr, w1, w2 = patches
#             # clear trajectory
#             line.set_data([], [])

#             # set body
#             x, y, theta = robot.state
#             circ.center = (x, y)

#             # set heading arrow
#             dx, dy = np.cos(theta), np.sin(theta)
#             arr.set_offsets([(x, y)])
#             arr.set_UVC(dx, dy)

#             # set wheels positions
#             # wheels are placed to the left and right of body circle
#             # compute wheel offsets perpendicular to heading
#             perp = np.array([-dy, dx])
#             left_center  = np.array([x, y]) + perp * (body_radius + wheel_width/2)
#             right_center = np.array([x, y]) - perp * (body_radius + wheel_width/2)

#             # set rectangles centered at these positions
#             for w, center in zip([w1, w2], [left_center, right_center]):
#                 # set bottom-left to center minus half size rotated by heading
#                 R = np.array([[np.cos(theta), -np.sin(theta)],
#                               [np.sin(theta),  np.cos(theta)]])
#                 # wheel rectangle oriented along heading
#                 w.set_width(wheel_length)
#                 w.set_height(wheel_width)
#                 w.angle = np.degrees(theta)
#                 # matplotlib Rectangle xy is lower-left corner; offset from center
#                 offset = R.dot(np.array([-wheel_length/2, -wheel_width/2]))
#                 w.set_xy(center + offset)

#             artists.extend([line, circ, arr, w1, w2])
#         return artists

#     def update(self, frame):
#         """
#         Advance each robot, update artists for this frame.
#         Called every frame by FuncAnimation.
#         """
#         artists = []
#         dt = self.interval / 1000.0  # convert ms to seconds

#         for line, patches, robot in zip(self.traj_lines, self.icon_patches, self.robots):
#             circ, arr, w1, w2 = patches

#             # advance robot state
#             robot.step(dt)
#             hist = np.array(robot.history)

#             # update trajectory line
#             x_hist, y_hist = hist[:, 0], hist[:, 1]
#             line.set_data(x_hist, y_hist)

#             # update body
#             x, y, theta = robot.state
#             circ.center = (x, y)

#             # update heading arrow
#             dx, dy = np.cos(theta), np.sin(theta)
#             arr.set_offsets([(x, y)])
#             arr.set_UVC(dx, dy)

#             # update wheels as in init_draw
#             perp = np.array([-dy, dx])
#             left_center  = np.array([x, y]) + perp * (body_radius + wheel_width/2)
#             right_center = np.array([x, y]) - perp * (body_radius + wheel_width/2)
#             for w, center in zip([w1, w2], [left_center, right_center]):
#                 w.set_angle(np.degrees(theta))
#                 offset = R.dot(np.array([-wheel_length/2, -wheel_width/2]))
#                 w.set_xy(center + offset)

#             artists.extend([line, circ, arr, w1, w2])

#         return artists

#     def run(self):
#         """
#         Launch the animation.
#         """
#         self.anim = FuncAnimation(
#             self.fig,
#             self.update,
#             frames=self.n_frames,
#             init_func=self.init_draw,
#             blit=True,
#             interval=self.interval
#         )
#         plt.show()

# # Example usage omitted for brevity



obst1 = np.array( [ [-1., -1.5], [-0.5, -1.5],\
        [-0.5, -1.], [-1., -1.], [-1, -1.5] ]) 
obst2 = np.array( [ [-1., 1.5], [-0.5, 1.5],\
        [-0.5, 1.], [-1., 1.], [-1, 1.5] ])

obstacles = [obst1, obst2]

robot1_state = [-2., 1.5, 0.]
robot2_state = [-2., -1., 0.]
robot1 = Robot(robot_id=1, initial_state=robot1_state, color='r')
robot2 = Robot(robot_id=2, initial_state=robot2_state, color='b')
robots = [robot1, robot2]

fieldx = (-3.0, 3.0)
fieldy = (-2, 2)
sim = SimulationVisualizer(robots=robots, static_obstacles=obstacles, fieldx=fieldx, fieldy=fieldy)
sim.draw_static_obstacles(obstacles=obstacles)
sim.init_icons()

plt.show()