import numpy as np

class Robot:
    """
    Represents a single moving robot in the simulator
    """
    def __init__(self, robot_id, color):
        self.id = robot_id
        self.state = NotImplemented
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