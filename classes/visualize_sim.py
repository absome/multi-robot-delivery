from .robot import Robot
from .detect_obstacle import DetectObstacle
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class SimViz:
    def __init__(self, robots, static_obstacles, field):
        self.robots = robots
        self.static_obstacles = static_obstacles
        self.field = field # ((-xlim, xlim), (-ylim, ylim))
        # Figure params
        self.fig = plt.figure(1)
        self.ax = plt.gca()
        self.ax.set(xlabel="x [m]", ylabel="y [m]", aspect="equal")
        self.ax.set(xlim=field[0], ylim=field[1])
        self.time_text = self.ax.text(0.78, 0.01, 't = 0 s', color = 'k', fontsize='large', 
                                horizontalalignment='left', verticalalignment='bottom', transform = self.ax.transAxes)
        
        self.init_static_obstacles(static_obstacles)
        self.robot_visuals = {r.id: self.init_robot_artists(r) for r in robots}

    def init_static_obstacles(self, obstacles):
        """
        Initializes static obstacles into figure
        """
        for obstacle in obstacles:
            self.ax.plot( obstacle[:,0], obstacle[:,1], '--r' )

    def init_robot_artists(self, robot):
        """
        Creates and returns artists for a single robot
        """
        # Robot params
        px, py, theta = robot.state_history[0]
        radius = robot.radius
        wheel_length = robot.wheel_length
        wheel_width = robot.wheel_width
        # Trajectory line, plots history for px and py
        line, = self.ax.plot([], [], linestyle='--', color=robot.color)
        # Robot body
        body = plt.Circle( (px, py), radius, color=robot.color)
        # Arrow
        arrow_pos =  [px, py]          
        arrow_direction = (radius*np.cos(theta), radius*np.sin(theta))

        # Wheels
        wheel_angle = [theta + 0., theta+np.pi]
        wheel_angle_degrees = [np.rad2deg(theta + 0.), np.rad2deg(theta+np.pi)]
        wh_x = [ px - radius*np.sin(i) - (wheel_length/2)*np.cos(i) + (wheel_width/2)*np.sin(i) for i in wheel_angle ]
        wh_y = [ py + radius*np.cos(i) - (wheel_length/2)*np.sin(i) - (wheel_width/2)*np.cos(i) for i in wheel_angle ]
        w1 = plt.Rectangle( (wh_x[0], wh_y[0]), wheel_length, wheel_width,
            angle=wheel_angle_degrees[0], color='k')
        w2 = plt.Rectangle( (wh_x[1], wh_y[1]), wheel_length, wheel_width,
                            angle=wheel_angle_degrees[1], color='k')
        # Add patch
        self.ax.add_patch(body)
        self.ax.add_patch(w1)
        self.ax.add_patch(w2)
        arrow = plt.quiver(arrow_pos[0], arrow_pos[1], 
                            arrow_direction[0], arrow_direction[1],
                            scale_units='xy', scale=1, color='k', width=0.1*radius)
        return {'line':line, 'body':body, 'arrow':arrow, 'wheels':(w1, w2)}
    
    def init_draw(self):
        # Add time text to plot
        self.time_txt.set_text('t = 0.0 s')        
        artists = [self.time_text]
        for robot in self.robots:
            robot_visuals = self.robot_visuals[robot.id]
            robot_visuals['line'].set_data([], [])
            # reset history
            robot.state_history = [robot.state_history[0]]
            artists += [robot_visuals['line'], robot_visuals['body'],
                        robot_visuals['arrow'], *robot_visuals['wheels']]
        return artists
    
    def update_robot_artists(self):
        """
        Update positions of the robot's artists according to the state and returns them
        """
        robot_artists = []
        for robot in self.robots:
            # Update trajectory line
            robot.step()
            state_history = robot.state_history
            visuals = self.robot_visuals[robot.id]
            visuals['line'].set_data(state_history[:,0], state_history[:,1])
            robot_artists.append(visuals['line'])
            # Update body position
            px, py, theta = robot.state[:]
            self.robot_visuals[robot.id]['body'].center = (px, py)
            robot_artists.append(self.robot_visuals[robot.id]['body'])
            # Update arrow position
            dx, dy = np.cos(theta), np.sin(theta)
            self.robot_visuals[robot.id]['arrow'].set_offsets([(px, py)])
            self.robot_visuals[robot.id]['arrow'].set_UVC(dx, dy)
            robot_artists.append(self.robot_visuals[robot.id]['arrow'])
            
            # Update wheel position
            wheel_length, wheel_width = robot.wheel_length, robot.wheel_width
            wheel_angle = [theta + 0., theta+np.pi]
            wheel_angle_degrees = [np.rad2deg(wheel_angle[0]),
                                np.rad2deg(wheel_angle[1])]
            R = np.array([ # rotation matrix for heading
                [np.cos(theta), -np.sin(theta)],
                [np.sin(theta),  np.cos(theta)]
            ])
            perp = np.array([-dy, dx])
            centers = [np.array([px,py]) + perp*(robot.radius+wheel_width/2), np.array([px,py]) - perp*(robot.radius+wheel_width/2)]
            for w, c in zip(self.robot_visuals[robot.id]['wheels'], centers):
                w.angle = np.degrees(theta)
                offset = R.dot(np.array([-wheel_length/2, -wheel_width/2]))
                w.set_xy(c + offset)
                robot_artists.append(w)

        return robot_artists
        
        
    def update_animation(self):
        """
        Updates figures animation with all moved artists
        """
        artists = []
        updated_robot_artists = self.update_robot_artists()
        artists += updated_robot_artists
        
        return artists
    
    def run(self):
        self.anim = FuncAnimation(
            self.fig,
            self.update_animation,
            init_func=self.init_draw,
            frames=self.n_frames,
            interval=self.interval,
            blit=True,
            repeat=True
        )
        plt.show()

## Setting up sensor updating points
# Initiate object detection
range_sensor = DetectObstacle( sensing_range, sensor_resolution)
range_sensor.register_obstacle_bounded( obst_vertices )
# At time step 0
# compute and plot sensor reading endpoint
distance_reading = range_sensor.get_sensing_data( robot_state[0], robot_state[1], robot_state[2])
obst_points = compute_sensor_endpoint(robot_state, distance_reading)
pl_sens, = sim_visualizer.ax.plot(obst_points[0], obst_points[1], '.') #, marker='X')
pl_txt = [sim_visualizer.ax.text(obst_points[0,i], obst_points[1,i], str(i)) for i in range(len(distance_reading))]
# For each time step, update sensor visualization
pl_sens.set_data(obst_points[0], obst_points[1])
for i in range(len(distance_reading)): pl_txt[i].set_position((obst_points[0,i], obst_points[1,i]))


# class SimulationVisualizer:
#     """
#     Simulator that visualizes robot movement and the robots environment
#     """
#     def __init__(self, robots, static_obstacles, field):
#         self.robots = robots
#         # FPS
#         self.interval = 100
#         self.n_frames = 200
#         self.sim_dt = 0.1
#         self.static_obstacles = static_obstacles
#         # Plot params
#         self.fig = plt.figure(1)
#         self.ax = plt.gca()
#         self.ax.set(xlabel="x [m]", ylabel="y [m]", aspect="equal")
#         self.ax.set(xlim=field[0], ylim=field[1])
        
#         # Time text
#         self.time_txt = self.ax.text(0.78, 0.01, 't = 0 s', color = 'k', fontsize='large', 
#                     horizontalalignment='left', verticalalignment='bottom', transform = self.ax.transAxes)
#         # Create artists for each robot
#         self.robot_visuals = {r.id: self.make_robot_artists(r) for r in robots}
    
#     def make_robot_artists(self, robot):
#         """
#         Creates and returns artists for a single robot
#         """
#         # Trajectory line, plots history for px and py
#         line, = self.ax.plot([], [], linestyle='--', color=robot.color)
#         # Robot body
#         px, py, theta = robot.state[:]
#         body = plt.Circle( (0, 0), robot.radius, color=robot.color)
#         self.ax.add_patch(body)
#         # Robot arrow
#         arrow = plt.quiver( 0, 0, 1, 0, 
#             scale_units='xy', scale=1, color='k', width=0.1*robot.radius)
#         # Wheels
#         wheel_angle = [theta + 0., theta+np.pi]
#         wheel_angle_degrees = [np.rad2deg(wheel_angle[0]),
#                                np.rad2deg(wheel_angle[1])]
#         w1 = plt.Rectangle( (0, 0), robot.wheel_length, robot.wheel_width,
#                             angle=wheel_angle_degrees[0], color='k')
#         w2 = plt.Rectangle( (0, 0), robot.wheel_length, robot.wheel_width,
#                             angle=wheel_angle_degrees[1], color='k')
        
#         return {'line':line, 'body':body, 'arrow':arrow, 'wheels':(w1, w2)}


    # def update_icon(self, artists, robot):
    #     px, py, theta = robot.state[:]
    #     # Body
    #     artists['body'].center = (px, py)
    #     # Arrow
    #     dx, dy = np.cos(theta), np.sin(theta)
    #     artists['arrow'].set_offsets([(px, py)])
    #     artists['arrow'].set_UVC(dx, dy)
    #     # Wheels
    #     wheel_length, wheel_width = robot.wheel_length, robot.wheel_width
    #     wheel_angle = [theta + 0., theta+np.pi]
    #     wheel_angle_degrees = [np.rad2deg(wheel_angle[0]),
    #                            np.rad2deg(wheel_angle[1])]
    #     R = np.array([ # rotation matrix for heading
    #         [np.cos(theta), -np.sin(theta)],
    #         [np.sin(theta),  np.cos(theta)]
    #     ])
    #     perp = np.array([-dy, dx])
    #     centers = [np.array([px,py]) + perp*(robot.radius+wheel_width/2), np.array([px,py]) - perp*(robot.radius+wheel_width/2)]
    #     for w, c in zip(artists['wheels'], centers):
    #         w.angle = np.degrees(theta)
    #         offset = R.dot(np.array([-wheel_length/2, -wheel_width/2]))
    #         w.set_xy(c + offset)

#     def init_draw(self):
#         artists = [self.time_txt]
#         for robot in self.robots:
#             robot_visuals = self.robot_visuals[robot.id]
#             # clear line trajectory
#             robot_visuals['line'].set_data([], [])
#             # reset history
#             robot.history = [robot.history[0]]
#             # update icon
#             self.update_icon(robot_visuals, robot.state)
#             artists += [robot_visuals['line'], robot_visuals['body'],
#                         robot_visuals['arrow'], *robot_visuals['wheels']]
#         return artists
    
    # def update(self, frame):
    #     artists = []
    #     t = frame * self.sim_dt
    #     self.time_txt.set_text(f't = {t:.1f} s'); artists.append(self.time_txt)
    #     for robot in self.robots:
    #         robot.step(self.sim_dt)
    #         hist = np.array(robot.history)
    #         v = self.robot_visuals[robot.id]
    #         # update trajectory
    #         v['line'].set_data(hist[:,0], hist[:,1]); artists.append(v['line'])
    #         # update icon
    #         self.update_icon(v, robot.state)
    #         artists += [v['body'], v['arrow'], *v['wheels']]
    #     return artists
    
#     def run(self):
#         # store the animation so resize callbacks get tracked
#         self.anim = FuncAnimation(
#             self.fig, self.update, init_func=self.init_draw,
#             frames=range(int(1/self.sim_dt)*10),
#             interval=self.sim_dt*1000,
#             blit=False, repeat=True, repeat_delay=500
#         )
#         plt.show()


# class SimulationVisualizer:
#     """
#     Handles visualization for a multi-robot system
#     """
#     def __init__(self, robots, static_obstacles, fieldx, fieldy):
#         # FPS, interval ms between frames and number of frames
#         self.interval = 100
#         self.n_frames = 200
        
#         # Plot parameters
#         self.fig = plt.figure(1)
#         self.ax = plt.gca()
#         self.ax.set(xlabel="x [m]", ylabel="y [m]")
#         xlim, ylim = fieldx, fieldy
#         print(xlim)
#         self.set_field(x_axis_range=xlim, y_axis_range=ylim)
#         self.ax.set_aspect('equal', adjustable='box', anchor='C')
#         plt.tight_layout()
        
#         # Set text to count simulationt time
#         self.time_txt = self.ax.text(0.78, 0.01, 't = 0 s', color = 'k', fontsize='large', 
#             horizontalalignment='left', verticalalignment='bottom', transform = self.ax.transAxes)
        
#         # Store obstacle icon and trajectory for each robot id
#         self.robot_visuals = {}
#         for robot in robots:
#             self.robot_visuals[robot.id] = {
#                 "line": None,
#                 "icon": []
#             }

#         self.static_obstacles = static_obstacles
#         self.moving_obstacles = []
#         self.robots = robots
        
#     def init_icons(self):
#         for robot in self.robots:
#             px = robot.state[0]
#             py = robot.state[1]
#             theta = robot.state[2]

#             # Robot size params
#             scale = 2
#             body_radius = 0.08 * scale
#             scale = 2
#             body_rad = 0.08 * scale # m
#             wheel_length = 0.1*scale
#             wheel_width = 0.02*scale
#             arrow_size = body_rad
            
#             # Robot body
#             body = plt.Circle( (px, py), body_rad, color=robot.color)

#             # Wheels
#             wheel_angle = [theta + 0., theta+np.pi]
#             wheel_angle_degrees = [np.rad2deg(i) for i in wheel_angle]
#             wh_x = [ px - body_rad*np.sin(i) - (wheel_length/2)*np.cos(i) + (wheel_width/2)*np.sin(i) for i in wheel_angle ]
#             wh_y = [ py + body_rad*np.cos(i) - (wheel_length/2)*np.sin(i) - (wheel_width/2)*np.cos(i) for i in wheel_angle ]
            
#             w1 = plt.Rectangle( (wh_x[0], wh_y[0]), wheel_length, wheel_width,
#                                angle=wheel_angle_degrees[0], color='k')
#             w2 = plt.Rectangle( (wh_x[1], wh_y[1]), wheel_length, wheel_width,
#                                angle=wheel_angle_degrees[1], color='k')
                
#             # Arrow
#             ar_st= [px, py] #[ px - (arrow_size/2)*np.cos(th), py - (arrow_size/2)*np.sin(th) ]
#             ar_d = (arrow_size*np.cos(theta), arrow_size*np.sin(theta))

        
#             line, = self.ax.plot([], [], linestyle='--', color=robot.color)

#             # Add patch
#             self.ax.add_patch(body)
#             self.ax.add_patch(w1)
#             self.ax.add_patch(w2)
#             # arrow drawn after for visibility
#             arrow = plt.quiver( ar_st[0], ar_st[1], ar_d[0], ar_d[1], 
#                 scale_units='xy', scale=1, color='k', width=0.1*arrow_size)
            
#             self.robot_visuals[robot.id]["line"] = line
#             self.robot_visuals[robot.id]["icon"] = [body, arrow, w1, w2]

#     def init_draw(self):
#         """
#         Initialize all artists to their starting positions.
#         Called once by FuncAnimation.
#         """
#         artists = []
#         for robot in self.robots:
#             line = self.robot_visuals[robot.id]["line"]
#             icon = self.robot_visuals[robot.id]["icon"]
#             body, arrow, w1, w2 = icon # unpacks robot visual characteristics
#             # reset line trajectory
#             line.set_data([], [])
#             artists.append(line)
            
#             # initial simulation time
#             self.time_txt.set_text('t = 0.0 s')
#             artists.append(self.time_txt)
            
#             # reset body position
#             px, py, theta = robot.state
#             body.center = (px, py)
#             artists.append(body)
            
#             # reset arrow position
#             dx, dy = np.cos(theta), np.sin(theta)
#             arrow.set_offsets([(px, py)])
#             arrow.set_UVC(dx, dy)
#             artists.append(arrow)
            
#             # reset wheel positions
#             R = np.array([ # rotation matrix for heading
#                 [np.cos(theta), -np.sin(theta)],
#                 [np.sin(theta),  np.cos(theta)]
#             ])
            
#             arrow_size = robot.radius

#             # Wheels
#             wheel_angle = [theta + 0., theta+np.pi]
#             wheel_angle_degrees = [np.rad2deg(i) for i in wheel_angle]
#             wh_x = [ px - robot.radius*np.sin(i) - (robot.wheel_length/2)*np.cos(i) + (robot.wheel_width/2)*np.sin(i) for i in wheel_angle ]
#             wh_y = [ py + robot.radius*np.cos(i) - (robot.wheel_length/2)*np.sin(i) - (robot.wheel_width/2)*np.cos(i) for i in wheel_angle ]
#             w1.set_width(robot.wheel_width)
#             w2.set_width(robot.wheel_width)
#             w1.set_height(robot.wheel_length)
#             w2.set_height(robot.wheel_length)
#             w1.angle = wheel_angle_degrees[0]
#             w2.angle = wheel_angle_degrees[1]
#             offset = R.dot(np.array([-self.wheel_length/2, -self.wheel_width/2]))
#             w1_center = np.array([wh_x[0], wh_y[0]])
#             w2_center = np.array([wh_x[1], wh_y[1]])
#             w1.set_xy(w1_center + offset)
#             w2.set_xy(w2_center + offset)
            
#             artists.extend([line, icon, arrow, w1, w2, self.time_txt])
#         return artists

#     def update(self, frame):
#         """
#         Advance each robot, update artists for this frame.
#         Called every frame by FuncAnimation.
#         """
#         artists = []
#         t = frame * 0.1
#         self.time_txt.set_text(f't = {t:.1f} s')
#         for robot in self.robots:
#             line = self.robot_visuals["line"]
#             icon = self.robot_visuals["icon"]
#             body, arrow, w1, w2 = icon # unpacks robot visual characteristics
            
#             # step the robot
#             robot.step(0.1)
#             hist = np.array(robot.history)
#             x_hist, y_hist = hist[:, 0], hist[:, 1]
#             line.set_data(x_hist, y_hist)
            
#             px, py, theta = robot.state
#             body.center = (px, py)

#             # update heading arrow
#             dx, dy = np.cos(theta), np.sin(theta)
#             arrow.set_offsets([(px, py)])
#             arrow.set_UVC(dx, dy)
            
#             # Wheels
#             scale = 2
#             body_radius = 0.08 * scale
#             scale = 2
#             body_rad = 0.08 * scale # m
#             wheel_length = 0.1*scale
#             wheel_width = 0.02*scale
#             arrow_size = body_rad

#             R = np.array([ # rotation matrix for heading
#                 [np.cos(theta), -np.sin(theta)],
#                 [np.sin(theta),  np.cos(theta)]
#             ])
#             wheel_angle = [theta + 0., theta+np.pi]
#             wheel_angle_degrees = [np.rad2deg(i) for i in wheel_angle]
#             wh_x = [ px - body_rad*np.sin(i) - (wheel_length/2)*np.cos(i) + (wheel_width/2)*np.sin(i) for i in wheel_angle ]
#             wh_y = [ py + body_rad*np.cos(i) - (wheel_length/2)*np.sin(i) - (wheel_width/2)*np.cos(i) for i in wheel_angle ]
#             w1.set_width(wheel_width)
#             w2.set_width(wheel_width)
#             w1.set_height(wheel_length)
#             w2.set_height(wheel_length)
#             w1.angle = wheel_angle_degrees[0]
#             w2.angle = wheel_angle_degrees[1]
#             offset = R.dot(np.array([-self.wheel_length/2, -self.wheel_width/2]))
#             w1_center = np.array([wh_x[0], wh_y[0]])
#             w2_center = np.array([wh_x[1], wh_y[1]])
#             w1.set_xy(w1_center + offset)
#             w2.set_xy(w2_center + offset)
            
#             artists.extend([line, icon, arrow, w1, w2, self.time_txt])
#         return artists
    
#     def set_field(self, x_axis_range, y_axis_range):
#         """
#         Sets the range for each axis, x and y on the graph
#         """
#         self.ax.set( xlim=x_axis_range, ylim = y_axis_range)
        
#     def show_goal(self):
#         raise NotImplementedError
    
#     def draw_robot(self, robot_state):
#         raise NotImplementedError
    

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
    
#     def draw_static_obstacles(self, obstacles):
#         for obstacle in obstacles:
#             self.ax.plot( obstacle[:,0], obstacle[:,1], '--r' )