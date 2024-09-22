import numpy as np 
import matplotlib.pyplot as plt
import os
from collections import defaultdict
from tqdm import tqdm

roadmap_dir = "obstacle_0/"
roadmap_path = os.environ['DIRTMP_PATH'] + "/resources/roadmaps/" + roadmap_dir
output_path = os.environ['DIRTMP_PATH'] + "/out/"

# Load vertices
vertices_raw = np.loadtxt(roadmap_path + "vertices.txt", delimiter=',')
vertices = {}
for v in vertices_raw:
    vertices[int(v[0])] = [v[1:].tolist()]

edges_raw = np.loadtxt(roadmap_path + "edges.txt", delimiter=',')
# Edge costs should be a dict with source and target as keys
edges = defaultdict(lambda: float('inf'))
max_cost = 0
for e in edges_raw:
    edges[int(e[0]), int(e[1])] = e[2]
    if e[2] > max_cost:
        max_cost = e[2]

# Create a colormap for the costs
cmap = plt.get_cmap('hot')

plt.figure(figsize=(10,10))
plt.scatter(vertices_raw[:,1], vertices_raw[:,2], marker='+')

for f in tqdm(os.listdir(roadmap_path)):
    if f.startswith("traj_"):
        traj = np.loadtxt(roadmap_path + f, delimiter=',')  
        source = int(f.split("_")[1])
        target = int(f.split("_")[2].split(".")[0])
        # colormap based on the cost
        cost = edges[source, target]
        color = cmap(cost/max_cost)
        plt.plot(traj[:,0], traj[:,1], color=color, alpha=0.5)
        # Arrow in the middle of the trajectory
        arrow = np.array([traj[int(len(traj)/2),0], traj[int(len(traj)/2),1]])
        plt.arrow(arrow[0], arrow[1], traj[int(len(traj)/2)+1,0] - arrow[0], traj[int(len(traj)/2)+1,1] - arrow[1], head_width=0.05, head_length=0.05, fc='black', ec='black')
plt.savefig(output_path + roadmap_dir[:-1] + ".png")