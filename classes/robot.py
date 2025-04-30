import numpy as np

class Robot:
    """
    Represents a single moving robot in the simulator
    """
    def __init__(self, initial_state, robot_id, color):
        self.id = robot_id
        self.state = initial_state # notimplemented
        self.goal_state = NotImplemented
        self.controller = NotImplemented
        self.color = color
    
        # Current internal state
        self.current_input = NotImplemented
        self.sensor_readings = NotImplemented
        self.caster_point = NotImplemented

        # Stored states
        self.state_history = NotImplemented
        self.input_history = NotImplemented
        
        print(f"Intialized robot with ID: {self.id}")
        
    def compute_control_input(self):
        raise NotImplementedError
    
    def get_sensing_data(self):
        raise NotImplementedError
    
    def get_caster_point(self):
        raise NotImplementedError
    
    
# class Robot:
#     """
#     Encapsulates robot state and dynamics.
#     state: np.ndarray of shape (3,) -> [x, y, theta]
#     dynamics: function(state: np.ndarray, dt: float) -> np.ndarray
#     color: fill color for visualization
#     """
#     def __init__(self, init_state, dynamics_func, color='C0'):
#         self.state = np.array(init_state, dtype=float)
#         self.dynamics = dynamics_func
#         self.history = [self.state.copy()]
#         self.color = color

#     def step(self, dt):
#         """
#         Advance the robot's state by dt using the dynamics function.
#         """
#         self.state = self.dynamics(self.state, dt)
#         self.history.append(self.state.copy())