import numpy as np
import matplotlib.pyplot as plt
import os

from matplotlib.patches import Circle

fname = os.environ['DIRTMP_PATH'] + "out/"
goal_state = np.array([9.,9.])
goal_radius = 0.5

plt.figure(figsize=(8, 8))
plt.xlim(-10,10)
plt.ylim(-10,10)

circle = Circle(goal_state, goal_radius, color='green')
plt.gca().add_patch(circle)

for f in os.listdir(fname):
    if f.endswith(".txt"):
        data = np.loadtxt(fname+f,delimiter=',')
        if f.startswith("tree"):
            plt.plot(data[:,0], data[:,1], color='black',linewidth=0.5)
        if f.startswith("solution"):
            plt.plot(data[:,0], data[:,1], color='red', linewidth=2)

plt.show()