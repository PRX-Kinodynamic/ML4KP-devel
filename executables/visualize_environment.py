import numpy as np 
import matplotlib.pyplot as plt
import yaml
import os
from matplotlib.patches import Rectangle

environment_file = os.environ["DIRTMP_PATH"]+"resources/input_files/environments/warehouse.yaml"

with open(environment_file, 'r') as stream:
    try:
        env_params = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

obstacles = env_params["environment"]["geometries"]
plt.figure()
for obstacle in obstacles:
    box_center = obstacle["config"]["position"][:2]
    box_dims = obstacle["collision_geometry"]["dims"][:2]
    rect = Rectangle((box_center[0]-box_dims[0]/2.0,box_center[1]-box_dims[1]/2.0),box_dims[0],box_dims[1],
    linewidth=1,edgecolor='r',facecolor='r')
    plt.gca().add_patch(rect)
plt.xlim(0,31)
plt.ylim(0,18)


# points_file = os.environ["DIRTMP_PATH"]+"out/1114/points.txt"

# points = np.loadtxt(points_file,delimiter=",")
# plt.scatter(points[:,0],points[:,1],c="b")
# for i in range(points.shape[0]):
#     # Column 3 is the angle. Draw an arrow from (x,y) to (x+cos(angle),y+sin(angle))
#     plt.arrow(points[i,0],points[i,1],np.cos(points[i,2]),np.sin(points[i,2]),color="black",head_width=0.2,head_length=0.2)

'''
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
'''
plt.savefig(os.environ['HOME']+'/Desktop/warehouse.png',bbox_inches='tight')
# plt.show()
