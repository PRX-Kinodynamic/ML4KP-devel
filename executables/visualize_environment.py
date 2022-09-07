import numpy as np 
import matplotlib.pyplot as plt
import yaml
import os
from matplotlib.patches import Rectangle

# environment_file = os.environ["DIRTMP_PATH"]+"resources/input_files/environments/narrow.yaml"
# environment_file = os.environ["DIRTMP_PATH"]+"resources/input_files/environments/maze.yaml"
# environment_file = os.environ["DIRTMP_PATH"]+"resources/input_files/environments/city.yaml"
environment_file = os.environ["DIRTMP_PATH"]+"resources/input_files/environments/simple_warehouse.yaml"
# environment_file = os.environ["DIRTMP_PATH"]+"resources/input_files/environments/rrt_star_obstacles.yaml"

with open(environment_file, 'r') as stream:
    try:
        env_params = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

obstacles = env_params["environment"]["geometries"]
plt.figure(figsize=(8,8))
for obstacle in obstacles:
    box_center = obstacle["config"]["position"][:2]
    box_dims = obstacle["collision_geometry"]["dims"][:2]
    rect = Rectangle((box_center[0]-box_dims[0]/2.0,box_center[1]-box_dims[1]/2.0),box_dims[0],box_dims[1],
    linewidth=1,edgecolor='r',facecolor='r')
    plt.gca().add_patch(rect)
plt.xlim(-14,14)
plt.ylim(-14,14)

vertices_fname = os.environ["DIRTMP_PATH"]+"out/vertices.txt"
vertices_raw = np.loadtxt(vertices_fname,delimiter=",")

vertices = {}
for i in range(vertices_raw.shape[0]):
    vertices[int(vertices_raw[i,0])] = vertices_raw[i,1:]

plt.scatter(vertices_raw[:,1],vertices_raw[:,2],marker='o',s=50)
# Label vertices with their idx
for i in range(vertices_raw.shape[0]):
    plt.annotate(str(int(vertices_raw[i,0])),(vertices_raw[i,1],vertices_raw[i,2]))

edges_fname = os.environ["DIRTMP_PATH"]+"out/edges.txt"
edges = np.loadtxt(edges_fname,delimiter=",")

for edge in edges:
    vertex_from = vertices[int(edge[0])]
    vertex_to = vertices[int(edge[1])]

    # Plot arrow from vertex_from to vertex_to
    # If edge[2] == 0, plot as a dotted line.
    if edge[2] == 0:
        plt.arrow(vertex_from[0],vertex_from[1],vertex_to[0]-vertex_from[0],vertex_to[1]-vertex_from[1],linestyle='dotted',
        head_width=0.25, head_length=0.25,color='red')
    else:
        plt.arrow(vertex_from[0],vertex_from[1],vertex_to[0]-vertex_from[0],vertex_to[1]-vertex_from[1],
        head_width=0.25, head_length=0.25, fc='k', ec='k')

plt.show()