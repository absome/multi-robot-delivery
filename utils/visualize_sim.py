import numpy as np
import matplotlib.pyplot as plt

class SimMobileRobot:
    def __init__(self):
        self.mode = None
        # Setup plt figure
        self.fig = plt.figure(1)
        self.ax = plt.gca()
        self.ax.set(xlabel="x[m]", ylabel="y[m]")
        self.ax.set_aspect('equal', adjustable='box', anchor='C')
        plt.tight_layout()
        
    def set_field(self):
        raise NotImplementedError
    
    def show_goal(self):
        raise NotImplementedError
    
    