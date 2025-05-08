import numpy as np
from .detect_obstacle import DetectObstacle
from cvxopt import matrix, solvers
import cvxopt.solvers

class Robot:
    """
    Represents a single moving robot in the simulator
    """
    def __init__(self, initial_state, goal_state, robot_id, color, params:list=None):
        self.id = robot_id
        self.state = initial_state.copy()
        self.goal_state = goal_state
        self.controller = NotImplemented
        self.color = color
        self.total_bots = None
        self.pairs = None
        
        # Current internal state
        self.current_input = np.array([0., 0., 0.])
        sensing_range, sensor_resolution = 1, np.pi/8
        self.sensor = DetectObstacle(sensing_range, sensor_resolution)
        self.caster_distance = 0.1 # distance of caster point along robot X axis

        # Stored states
        self.state_history = np.array([self.state])
        self.input_history = NotImplemented
        
        # Other robots (used for QP control)
        self.other_robots = []
        
        if params:
            scale = 2
            self.radius, self.wheel_length, self.wheel_width, self.sensing_range, self.sensor_resolution = params
            self.radius *= scale
            self.wheel_length *= scale
            self.wheel_width *= scale
            
        else:
            scale = 2
            self.radius = 0.08 * scale
            self.wheel_length = 0.1*scale
            self.wheel_width = 0.02*scale  
            self.sensing_range = 1
            self.sensor_resolution = np.pi/8      
        
        print(f"Intialized robot with ID: {self.id}")

    def compute_ugtg(self, caster=False):
        """
        Computes go-to-goal control input that moves the
        robot towards the goal direction
        """
        # initial numpy array for [v, omega]
        current_input = np.array([0., 0.]) 
        # Constants
        v_0 = 0.5
        beta = 1.0
        
        # Unpack goal states
        desired_state = self.goal_state
        px_d, py_d, theta_d = desired_state[:]
        
        # Unpack robot states
        px, py, theta = self.state[:]
        
        py_diff = py_d - py
        px_diff = px_d - px
        theta_d = np.arctan2(py_diff, px_diff)

        # Compute gain
        error = desired_state - self.state
        error_norm = np.linalg.norm(error)
        k_g = v_0 * (1 - np.exp(-beta * error_norm)) / error_norm
        k_theta = 2*k_g
        
        # Compute angular velocity
        error_th = theta_d - theta
        error_th = np.arctan2(np.sin(error_th), np.cos(error_th))
        w = k_theta*(error_th)
        # Calculate forward velocity of robot
        v = k_g*np.sqrt(px_diff**2 + py_diff**2)
        current_input = np.array([v, w])
        
        if caster:
            sx, sy = self.caster_xy()
            gx, gy = self.goal_state[:2]
            # caster linear velocity
            ux, uy = k_g*(gx - sx), k_g*(gy - sy)
            v = np.cos(theta)*ux + np.sin(theta)*uy
            w = (-np.sin(theta)/self.caster_distance)*ux + (np.cos(theta)/self.caster_distance)*uy
            current_input[0] = v
            current_input[1] = w

        return current_input
    
    def register_other_robots(self, other_robots):
        """
        Takes in all intialized robots and saves only those that are not
        the current robot into a list
        """
        self.other_robots = [r for r in other_robots if r.id != self.id]
        
    def compute_qp(self):
        """
        Centralised CBF-QP for 4 planar robots with one circular obstacle.
        Each robot must keep rigid formation and avoid the obstacle.
        """
        robots = self.total_bots
        pairs = self.pairs
        N = len(robots)                # should be 4
        assert N == 4, "code below assumes 4 robots"

        # ---------- nominal controls ----------
        u_gtg = np.vstack([r.compute_ugtg(caster=True)[:2] for r in robots])   # (4,2)
        u_gtg_flat = u_gtg.reshape(-1)                                         # (8,)

        Q  = 2*np.eye(2*N)                     # 8×8
        c  = -2*u_gtg_flat                     # 8×1

        # ---------- barrier parameters ----------
        obs_pos   = np.array([0.5, 2])
        R_s       = 0.5
        gamma_f   = 0.5
        gamma_o   = 0.2
        eps       = 0.01
        d_ref     = 1.5

        # ---------- how many constraints ----------
        n_edge_constraints = len(pairs)*2
        n_obst_constraints = N
        n_constr = n_edge_constraints + n_obst_constraints

        H = np.zeros((n_constr, 2*N))
        b = np.zeros((n_constr, 1))
        row = 0

        # ---------- formation constraints ----------
        for (i_id, j_id) in pairs:
            # make indices 0-based
            i, j = i_id-1, j_id-1

            s_i = robots[i].caster_xy()
            s_j = robots[j].caster_xy()
            diff = s_i - s_j
            dij2 = diff @ diff

            # upper-distance barrier
            h1 = -(dij2) + (d_ref + eps)**2
            H[row, 2*i:2*i+2] =  2*diff
            H[row, 2*j:2*j+2] = -2*diff
            b[row, 0]         =  gamma_f * h1
            row += 1

            # lower-distance barrier
            h2 =  dij2 - (d_ref - eps)**2
            H[row, 2*i:2*i+2] = -2*diff
            H[row, 2*j:2*j+2] =  2*diff
            b[row, 0]         =  gamma_f * h2
            row += 1

        # ---------- obstacle constraints ----------
        for k in range(N):
            s_k = robots[k].caster_xy()
            diff_o = s_k - obs_pos
            h_obs  = diff_o @ diff_o - R_s**2
            H[row, 2*k:2*k+2] = -2*diff_o
            b[row, 0]         =  gamma_o * h_obs
            row += 1

        # ---------- solve ----------
        Q_mat = cvxopt.matrix(Q, tc='d')
        c_mat = cvxopt.matrix(c, (2*N,1), tc='d')
        H_mat = cvxopt.matrix(H, tc='d')
        b_mat = cvxopt.matrix(b, tc='d')

        cvxopt.solvers.options['show_progress'] = False
        sol = cvxopt.solvers.qp(Q_mat, c_mat, H_mat, b_mat)

        caster_inputs = np.array(sol['x']).reshape(N,2)     # one row per robot
        r_index = self.id - 1
        caster_input = caster_inputs[r_index]
        
        ux, uy = caster_input[:]
        theta = self.state[2]
        l = self.caster_distance
        v = np.cos(theta)*ux + np.sin(theta)*uy
        w = (-np.sin(theta)/l)*ux + (np.cos(theta)/l)*uy
        current_input = np.array([v, w])
        
        return current_input*5


    
    def get_sensing_data(self):
        raise NotImplementedError
    
    def compute_sensor_endpoint(self):
        """
        Template explanation
        
        Args:
        Returns:
            obst_points[:2,:]
                Template explanation
            distance_reading::[np.array]
                NumpyArray of length M, containing the distance to nearest
                obstacle for each ray (or max range for no hits)
        """
        # Get distance reading
        px, py, theta = self.state[:]
        distance_reading = self.sensor.get_sensing_data(px, py, theta)
        # assuming sensor position is in the robot's center
        sens_N = round(2*np.pi/self.sensor_resolution)
        sensors_theta = [i*2*np.pi/sens_N for i in range(sens_N)]
        obst_points = np.zeros((3,sens_N))

        R_WB = np.array([ [np.cos(self.state[2]), -np.sin(self.state[2]), self.state[0] ], \
            [np.sin(self.state[2]),  np.cos(self.state[2]), self.state[1] ], [0, 0, 1] ])
        for i in range(sens_N):
            R_BS = np.array([ [np.cos(sensors_theta[i]), -np.sin(sensors_theta[i]), 0 ], \
                [np.sin(sensors_theta[i]),  np.cos(sensors_theta[i]), 0 ], [0, 0, 1] ])
            temp = R_WB @ R_BS @ np.array([distance_reading[i], 0, 1])
            obst_points[:,i] = temp

        return obst_points[:2,:], distance_reading
    
    
    def register_obstacles(self, obstacles):
        """
        Stores the obstacles in the environment that the robot should sense
        """
        for obstacle in obstacles:
            self.sensor.register_obstacle_bounded(obstacle)

    # def compute_control_input(self, dt):
    #     """
    #     Advance the robot's state by dt using the dynamics function.
    #     """
    #     self.state = self.dynamics_func(self.state, dt)
    #     self.history.append(self.state.copy())

    def caster_xy(self):
        """
        Used to get position coordinations of caster point ahead
        of the robot body
        """
        px, py, theta = self.state[:]
        l = self.caster_distance
        sx = px + l*np.cos(theta)
        sy = py + l*np.sin(theta)
        caster_xy = np.array([sx, sy])
        return caster_xy

    def step(self, time_step):
        """
        Update new state of robot at time step t+1
        """
        current_input = self.compute_qp()
        self.current_input = current_input # store current_input
        v, w = current_input
        # Correctly compute delta based on current orientation
        dx = v * np.cos(self.state[2]) * time_step
        dy = v * np.sin(self.state[2]) * time_step
        dtheta = w * time_step
        
        new_state = self.state.copy()
        new_state[0] += dx
        new_state[1] += dy
        new_state[2] += dtheta
        new_state[2] = ( (new_state[2] + np.pi) % (2*np.pi) ) - np.pi  # Wrap angle
        
        self.state = new_state
        self.state_history = np.vstack([self.state_history, self.state])
    
    def add_neighbours(self, robots:list, pairs:list):
        self.total_bots = robots
        self.pairs = pairs
        