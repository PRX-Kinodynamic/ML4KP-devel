import numpy as np 
import matplotlib.pyplot as plt
import yaml
import os
from matplotlib.patches import Rectangle

# mode = "edges"
mode = "trajs"
# robot_dims = [1.1,0.842]
robot_dims = [.9,0.6]
diag_len = 0.25 * np.sqrt(robot_dims[0]**2 + robot_dims[1]**2)

# environment_file = os.environ["DIRTMP_PATH"]+"resources/input_files/environments/bar.yaml"
environment_file = os.environ["DIRTMP_PATH"]+"resources/input_files/environments/simple_obstacle.yaml"

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
plt.xlim(-11,11)
plt.ylim(-11,11)

roadmap_dir = os.environ["DIRTMP_PATH"] + "out/ablation/"

for fname in os.listdir(roadmap_dir):
    if fname.endswith(".txt"):
        if mode == "trajs" and fname.startswith("traj"):
            traj = np.loadtxt(roadmap_dir+fname,delimiter=",")
            if len(traj) == 0: continue
            plt.plot(traj[:,0],traj[:,1],color='black')
            # Plot an arrow in the middle of the trajectory
            mid = int(len(traj)/2)
            plt.arrow(traj[mid,0],traj[mid,1],traj[mid+10,0]-traj[mid,0],traj[mid+10,1]-traj[mid,1],color='black',width=0.1)

vertices_raw = np.loadtxt(roadmap_dir+"vertices.txt",delimiter=",")

vertices = {}
for i in range(vertices_raw.shape[0]):
    vertices[int(vertices_raw[i,0])] = vertices_raw[i,1:]

# plt.scatter(vertices_raw[:,1],vertices_raw[:,2],marker='o',s=100)
for k,v in vertices.items():
    rectangle_corner = np.array([v[0]-diag_len*np.cos(0.25*np.pi+v[2]),
                                v[1]-diag_len*np.sin(0.25*np.pi+v[2])])
    rect = Rectangle((rectangle_corner[0],rectangle_corner[1]),
                robot_dims[0],robot_dims[1],
                edgecolor='purple',facecolor='purple',angle=180.*v[2]/np.pi)
    plt.gca().add_patch(rect)
    # Label vertices with their idx
    # for i in range(vertices_raw.shape[0]):
    #     plt.annotate(str(int(vertices_raw[i,0])),(vertices_raw[i,1],vertices_raw[i,2]))

edges_fname = roadmap_dir+"edges.txt"
edges = np.loadtxt(edges_fname,delimiter=",")

if mode == "edges":
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
plt.savefig('foo.png')