import numpy as np 
import matplotlib.pyplot as plt
import yaml
import os
from matplotlib.patches import Rectangle

robot_dims = [.9,0.6]
diag_len = 0.25 * np.sqrt(robot_dims[0]**2 + robot_dims[1]**2)

# environment_file = os.environ["DIRTMP_PATH"]+"resources/input_files/environments/bar.yaml"
environment_file = os.environ["DIRTMP_PATH"]+"resources/input_files/environments/landmark.yaml"
 
with open(environment_file, 'r') as stream:
    try:
        env_params = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

obstacles = env_params["environment"]["geometries"]


roadmap_dir = os.environ["DIRTMP_PATH"] + "out/ablation/drrm100/"

vertices_raw = np.loadtxt(roadmap_dir+"vertices.txt",delimiter=",")

vertices = {}
for i in range(vertices_raw.shape[0]):
    vertices[int(vertices_raw[i,0])] = vertices_raw[i,1:]

paths_prefix = "path"
plt.figure(figsize=(8,8))


for fname in os.listdir(roadmap_dir):
    if fname.startswith(paths_prefix) and fname.endswith(".txt"):
        path = np.loadtxt(roadmap_dir+fname,delimiter=",")
        for obstacle in obstacles:
            box_center = obstacle["config"]["position"][:2]
            box_dims = obstacle["collision_geometry"]["dims"][:2]
            rect = Rectangle((box_center[0]-box_dims[0]/2.0,box_center[1]-box_dims[1]/2.0),box_dims[0],box_dims[1],
            linewidth=1,edgecolor='r',facecolor='r')
            plt.gca().add_patch(rect)
        plt.xlim(-11,11)
        plt.ylim(-11,11)
        plt.plot(path[:,0],path[:,1],color='black')
        cost = np.floor((path.shape[0]-1)*0.1)
        # Display plot title as path[0][:3] -> path[-1][:3] nicely formatted
        plt.title("Path from "+str(path[0][:3])+" to "+str(path[-1][:3]) + " with cost "+str(cost))
        # Plot an arrow at the middle of the path
        plt.arrow(path[int(path.shape[0]/2),0],path[int(path.shape[0]/2),1],path[int(path.shape[0]/2)+10,0]-path[int(path.shape[0]/2),0],path[int(path.shape[0]/2)+10,1]-path[int(path.shape[0]/2),1],head_width=0.25, head_length=0.25, fc='k', ec='k')
        plt.savefig(roadmap_dir+fname+".png")
        plt.clf()

# plt.figure(figsize=(8,8))
# with open(paths_fname, 'r') as f:
#     for l in f:
#         plt.xlim(-11,11)
#         plt.ylim(-11,11)
#         for obstacle in obstacles:
#             box_center = obstacle["config"]["position"][:2]
#             box_dims = obstacle["collision_geometry"]["dims"][:2]
#             rect = Rectangle((box_center[0]-box_dims[0]/2.0,box_center[1]-box_dims[1]/2.0),box_dims[0],box_dims[1],
#             linewidth=1,edgecolor='r',facecolor='r')
#             plt.gca().add_patch(rect)
#         # Get space separated list of vertices
#         path = [int(e) for e in l.split(" ")[:-1]]
#         # Reverse the path
#         path = path[::-1]
#         # Plot those vertices
#         for k in path:
#             v = vertices[k]
#             rectangle_corner = np.array([v[0]-diag_len*np.cos(0.25*np.pi+v[2]),
#                                             v[1]-diag_len*np.sin(0.25*np.pi+v[2])])
#             rect = Rectangle((rectangle_corner[0],rectangle_corner[1]),
#                         robot_dims[0],robot_dims[1],
#                         edgecolor='purple',facecolor='purple',angle=180.*v[2]/np.pi)
#             plt.gca().add_patch(rect)

#         # Iterate through pairs of vertices
#         for i in range(len(path)-1):
#             traj_fname = roadmap_dir+"traj_"+str(path[i])+"_"+str(path[i+1])+".txt"
#             traj = np.loadtxt(traj_fname,delimiter=",")
#             plt.plot(traj[:,0],traj[:,1],color='black')
#             mid = int(traj.shape[0]/2)
#             plt.arrow(traj[mid,0],traj[mid,1],traj[mid+10,0]-traj[mid,0],traj[mid+10,1]-traj[mid,1],color='black',width=0.1)

#         plt.savefig("/Users/aravind/Downloads/"+str(path[0])+"_"+str(path[-1])+".png")
#         plt.clf()