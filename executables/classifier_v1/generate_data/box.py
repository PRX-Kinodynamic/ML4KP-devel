import numpy as np 
from geometry import Geometry
from matplotlib.patches import Rectangle, Circle

class Box(Geometry):
    def __init__(self, pos, size, euler=np.array([0, 0, 0])):
        super().__init__()
        self.pos = pos
        self.half_size = size
        self.euler = euler
    
    def update_pose(self, pos, euler):
        self.pos = pos
        self.euler = euler

    def get_extents(self):
        return self.pos - self.half_size, self.pos + self.half_size
        
    def plot(self, ax, movable=False):
        x, y = self.pos[0], self.pos[1]
        w, h = self.half_size[0], self.half_size[1]
        angle = self.euler[2]
        ax.add_patch(Rectangle((x-w, y-h), 2*w, 2*h, angle=np.rad2deg(angle), fill=True, rotation_point='center', color='red' if not movable else 'yellow'))