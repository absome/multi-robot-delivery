from .robot import Robot
from .detect_obstacle import DetectObstacle
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class SimViz:
    def __init__(self, robots, static_obstacles, field):
        # Sim params
        self.Ts = 0.01 # time step, update simulation every 10ms
        self.interval = 20 #50fps
        self.n_frames = 2000
        
        self.robots = robots
        self.static_obstacles = static_obstacles
        self.field = field # ((-xlim, xlim), (-ylim, ylim))
        # Figure params
        self.fig, self.ax = plt.subplots()
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
        self.time_text.set_text('t = 0.0 s')        
        artists = [self.time_text]
        for robot in self.robots:
            robot_visuals = self.robot_visuals[robot.id]
            robot_visuals['line'].set_data([], [])
            # reset history
            robot.state_history = [robot.state_history[0]]
            artists += [robot_visuals['line'], robot_visuals['body'],
                        robot_visuals['arrow'], *robot_visuals['wheels']]
        return artists
    
    def update_robot_artists(self, time_step):
        """
        Update positions of the robot's artists according to the state and returns them
        """
        robot_artists = []
        for robot in self.robots:
            # Update trajectory line
            robot.step(time_step)
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
        
        
    def update_animation(self, frame):
        """
        Updates figures animation with all moved artists
        """
        t = frame*self.Ts
        self.time_text.set_text(f't = {t:.1f} s')
        artists = [self.time_text]
        updated = self.update_robot_artists(self.Ts)
        artists.extend(updated)
        
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