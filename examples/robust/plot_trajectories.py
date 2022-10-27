import numpy as np 
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import os 

out_dir = os.environ["DIRTMP_PATH"] + "out/robust/"
goal_state = [5., 0.]
goal_radius = 0.5

plt.figure(figsize=(8, 8))
plt.xlim(-8,8)
plt.ylim(-8,8)

# Plot goal
plt.gca().add_patch(Circle(goal_state, goal_radius, color='g', alpha=0.5))

end_points = []

for fname in os.listdir(out_dir):
    if fname.endswith(".txt") and fname.startswith("traj"):
        data = np.loadtxt(out_dir + fname,delimiter=',')
        plt.plot(data[:,0], data[:,1], color='black')
        end_points.append(data[-1,:])
        plt.scatter(data[-1,0], data[-1,1], color='red',marker='x')

end_points = np.array(end_points)
# plt.scatter(np.mean(end_points[:,0]), np.mean(end_points[:,1]), color='blue',marker='o')

'''
hull = np.loadtxt(out_dir + "hull.txt",delimiter=',')
plt.plot(hull[:,0], hull[:,1], color='black')
# Plot a line from last point to first point
plt.plot([hull[-1,0],hull[0,0]], [hull[-1,1],hull[0,1]], color='black')
'''

plt.show()