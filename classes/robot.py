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

        # Stored states
        self.state_history = NotImplemented
        self.input_history = NotImplemented
        
    