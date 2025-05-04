import numpy as np
from .detect_obstacle import DetectObstacle

class Robot:
    """
    Represents a single moving robot in the simulator
    """
    def __init__(self, initial_state, robot_id, color, dynamics_func, params:list=None):
        self.id = robot_id
        self.state = initial_state.copy()
        self.goal_state = NotImplemented
        self.controller = NotImplemented
        self.color = color
    
        # Current internal state
        self.current_input = NotImplemented
        self.sensor = DetectObstacle(self.sensing_range, self.sensor_resolution)
        self.caster_point = NotImplemented

        # Stored states
        self.state_history = np.array([self.state])
        self.input_history = NotImplemented
        
        self.dynamics_func = dynamics_func        
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
        
    
    def compute_control_input(self):
        raise NotImplementedError
    
    def get_sensing_data(self):
        raise NotImplementedError
    
    def get_caster_point(self):
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

    def compute_control_input(self, dt):
        """
        Advance the robot's state by dt using the dynamics function.
        """
        self.state = self.dynamics_func(self.state, dt)
        self.history.append(self.state.copy())

