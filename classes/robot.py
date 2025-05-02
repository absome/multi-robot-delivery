import numpy as np

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
        self.sensor_readings = NotImplemented
        self.caster_point = NotImplemented

        # Stored states
        self.state_history = np.array([self.state])
        self.input_history = NotImplemented
        
        self.dynamics_func = dynamics_func        
        if params:
            scale = 2
            self.radius, self.wheel_length, self.wheel_width = params
            self.radius *= scale
            self.wheel_length *= scale
            self.wheel_width *= scale
        else:
            scale = 2
            self.radius = 0.08 * scale
            self.wheel_length = 0.1*scale
            self.wheel_width = 0.02*scale        
        
        print(f"Intialized robot with ID: {self.id}")
        
    
    def compute_control_input(self):
        raise NotImplementedError
    
    def get_sensing_data(self):
        raise NotImplementedError
    
    def get_caster_point(self):
        raise NotImplementedError

    def step(self, dt):
        """
        Advance the robot's state by dt using the dynamics function.
        """
        self.state = self.dynamics_func(self.state, dt)
        self.history.append(self.state.copy())

