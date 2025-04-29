import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class SimulationVisualizer:
    """
    Handles visualization for a multi-robot system
    """
    def __init__(self, field_x=(-3,3), field_y=(-2,2)):
        # Plot parameters
        self.fig = plt.figure(1)
        self.ax = plt.gca()
        self.ax.set(xlabel="x [m]", ylabel="y [m]")
        self.set_field(x_axis_range=field_x, y_axis_range=field_y)
        self.ax.set_aspect('equal', adjustable='box', anchor='C')
        plt.tight_layout()
        # Set text to count simulationt ime
        self.time_txt = self.ax.text(0.78, 0.01, 't = 0 s', color = 'k', fontsize='large', 
            horizontalalignment='left', verticalalignment='bottom', transform = self.ax.transAxes)
        # Obstacles
        self.static_obstacles = []
        self.moving_obstacles = []
        
    def init_draw(self):
        """
        For each robot, stores its visual characteristics into an array
        and returns that array
        """
        robots = [] #temp, remove later
        # Store robot visualization for each robot
        artists = []
        
        for line, circle, arrow, robot in zip(self.trajectory_lines,
                                          self.patch_circle,
                                          self.patch_arrow,
                                          self.robots):
            line.set_data([], [])
            x, y, theta = robot.state
            circle_center = (x, y)
            arrow_x, arrow_y = np.cos(theta), np.sin(theta)
            artists.extend([line, circle, arrow])
        
        return artists
    

    def set_field(self, x_axis_range, y_axis_range):
        """
        Sets the range for each axis, x and y on the graph
        """
        self.ax.set(xlim=x_axis_range, ylim=y_axis_range)
        
    def show_goal(self):
        raise NotImplementedError
    
    def draw_robot(self, robot_state):
        raise NotImplementedError


########## TEMPLATE ###################
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class Robot:
    """
    Encapsulates robot state and dynamics.
    state: np.ndarray of shape (3,) -> [x, y, theta]
    dynamics: function(state: np.ndarray, dt: float) -> np.ndarray
    color: fill color for visualization
    """
    def __init__(self, init_state, dynamics_func, color='C0'):
        self.state = np.array(init_state, dtype=float)
        self.dynamics = dynamics_func
        self.history = [self.state.copy()]
        self.color = color

    def step(self, dt):
        """
        Advance the robot's state by dt using the dynamics function.
        """
        self.state = self.dynamics(self.state, dt)
        self.history.append(self.state.copy())


class Simulation:
    """
    Manages multiple robots and animates their trajectories using FuncAnimation.
    Each robot is drawn as a filled circle with two side wheels and a heading arrow.
    """
    def __init__(self, robots, field=((-2, 2), (-2, 2)), interval=100, n_frames=200):
        self.robots = robots
        self.interval = interval  # ms between frames
        self.n_frames = n_frames

        # Setup figure and axes
        self.fig, self.ax = plt.subplots()
        self.ax.set(xlabel='x [m]', ylabel='y [m]', aspect='equal')
        xlim, ylim = field
        self.ax.set(xlim=xlim, ylim=ylim)

        # Prepare artists containers
        self.traj_lines = []      # Line2D for each robot trajectory
        self.icon_patches = []    # list of lists: [circle, arrow, wheel1, wheel2]

        # Initialize artists for each robot
        for robot in self.robots:
            # trajectory line
            line, = self.ax.plot([], [], linestyle='--', color=robot.color)

            # draw body circle (filled)
            body_radius = 0.1
            circ = plt.Circle((0, 0), body_radius, color=robot.color, alpha=0.5)
            self.ax.add_patch(circ)

            # heading arrow
            arr = self.ax.quiver(0, 0, 1, 0, scale_units='xy', scale=1, color='k', width=0.02)

            # wheels: two small rectangles perpendicular to heading
            wheel_length = 0.12
            wheel_width = 0.03
            # placeholders: actual positions set in init_draw
            w1 = plt.Rectangle((0, 0), wheel_length, wheel_width, angle=0, color='k')
            w2 = plt.Rectangle((0, 0), wheel_length, wheel_width, angle=0, color='k')
            self.ax.add_patch(w1)
            self.ax.add_patch(w2)

            self.traj_lines.append(line)
            self.icon_patches.append([circ, arr, w1, w2])

    def init_draw(self):
        """
        Initialize all artists to their starting positions.
        Called once by FuncAnimation.
        """
        artists = []
        for line, patches, robot in zip(self.traj_lines, self.icon_patches, self.robots):
            circ, arr, w1, w2 = patches
            # clear trajectory
            line.set_data([], [])

            # set body
            x, y, theta = robot.state
            circ.center = (x, y)

            # set heading arrow
            dx, dy = np.cos(theta), np.sin(theta)
            arr.set_offsets([(x, y)])
            arr.set_UVC(dx, dy)

            # set wheels positions
            # wheels are placed to the left and right of body circle
            # compute wheel offsets perpendicular to heading
            perp = np.array([-dy, dx])
            left_center  = np.array([x, y]) + perp * (body_radius + wheel_width/2)
            right_center = np.array([x, y]) - perp * (body_radius + wheel_width/2)

            # set rectangles centered at these positions
            for w, center in zip([w1, w2], [left_center, right_center]):
                # set bottom-left to center minus half size rotated by heading
                R = np.array([[np.cos(theta), -np.sin(theta)],
                              [np.sin(theta),  np.cos(theta)]])
                # wheel rectangle oriented along heading
                w.set_width(wheel_length)
                w.set_height(wheel_width)
                w.angle = np.degrees(theta)
                # matplotlib Rectangle xy is lower-left corner; offset from center
                offset = R.dot(np.array([-wheel_length/2, -wheel_width/2]))
                w.set_xy(center + offset)

            artists.extend([line, circ, arr, w1, w2])
        return artists

    def update(self, frame):
        """
        Advance each robot, update artists for this frame.
        Called every frame by FuncAnimation.
        """
        artists = []
        dt = self.interval / 1000.0  # convert ms to seconds

        for line, patches, robot in zip(self.traj_lines, self.icon_patches, self.robots):
            circ, arr, w1, w2 = patches

            # advance robot state
            robot.step(dt)
            hist = np.array(robot.history)

            # update trajectory line
            x_hist, y_hist = hist[:, 0], hist[:, 1]
            line.set_data(x_hist, y_hist)

            # update body
            x, y, theta = robot.state
            circ.center = (x, y)

            # update heading arrow
            dx, dy = np.cos(theta), np.sin(theta)
            arr.set_offsets([(x, y)])
            arr.set_UVC(dx, dy)

            # update wheels as in init_draw
            perp = np.array([-dy, dx])
            left_center  = np.array([x, y]) + perp * (body_radius + wheel_width/2)
            right_center = np.array([x, y]) - perp * (body_radius + wheel_width/2)
            for w, center in zip([w1, w2], [left_center, right_center]):
                w.set_angle(np.degrees(theta))
                offset = R.dot(np.array([-wheel_length/2, -wheel_width/2]))
                w.set_xy(center + offset)

            artists.extend([line, circ, arr, w1, w2])

        return artists

    def run(self):
        """
        Launch the animation.
        """
        self.anim = FuncAnimation(
            self.fig,
            self.update,
            frames=self.n_frames,
            init_func=self.init_draw,
            blit=True,
            interval=self.interval
        )
        plt.show()

# Example usage omitted for brevity
