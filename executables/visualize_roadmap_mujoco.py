import numpy as np 
import matplotlib.pyplot as plt
import os
from matplotlib.patches import Rectangle
from xml.dom import minidom

def quat2euler(quat):
    # Function that converts a quaternion [w,x,y,z] to euler angles [roll,pitch,yaw]
    # Source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    # Assumes that the quaternion is normalized
    w,x,y,z = quat
    roll = np.arctan2(2*(w*x+y*z),1-2*(x**2+y**2))
    pitch = np.arcsin(2*(w*y-z*x))
    yaw = np.arctan2(2*(w*z+x*y),1-2*(y**2+z**2))
    return yaw

robot_dims = [.45,0.31]
diag_len = 0.25 * np.sqrt(robot_dims[0]**2 + robot_dims[1]**2)
use_quat = True

environment_file = os.environ["DIRTMP_PATH"]+"resources/models/mujoco/indoor.xml"
# environment_file = os.environ["DIRTMP_PATH"]+"resources/models/mujoco/mushr_terrain.xml"

plt.figure(figsize=(8,8))
xmldoc = minidom.parse(environment_file)
# Get all <body> <geom> tags
body_list = xmldoc.getElementsByTagName('body')
for body in body_list:
    geom_list = body.getElementsByTagName('geom')
    for geom in geom_list:
        if geom.attributes['type'].value == "box":
            angle = 0
            if 'euler' in geom.attributes.keys():
                angle = np.degrees([float(x) for x in geom.attributes['euler'].value.split()][2])
            box_center = [float(x) for x in geom.attributes['pos'].value.split()]
            box_dims   = [float(x) for x in geom.attributes['size'].value.split()]
            rect = Rectangle((box_center[0]-box_dims[0], box_center[1]-box_dims[1]), 2*box_dims[0], 2*box_dims[1], \
                color='red', angle=angle, rotation_point='center')
            plt.gca().add_patch(rect)
plt.xlim(-11,11)
plt.ylim(-11,11)

# '''
roadmap_dir = os.environ["DIRTMP_PATH"] + "out/indoor/"
# roadmap_dir = "/Users/aravind/Downloads/mujoco_roadmap/"

for fname in os.listdir(roadmap_dir):
    if fname.endswith(".txt"):
        if fname.startswith("traj"):
            traj = np.loadtxt(roadmap_dir+fname,delimiter=",")
            if len(traj) == 0: continue
            plt.plot(traj[:,0],traj[:,1],color='black')
            # Plot an arrow in the middle of the trajectory
            mid = int(len(traj)/2)
            plt.arrow(traj[mid,0],traj[mid,1],traj[mid+10,0]-traj[mid,0],traj[mid+10,1]-traj[mid,1],color='black',width=0.1)

vertices_raw = np.loadtxt(roadmap_dir+"points.txt",delimiter=",")

# vertices = {}
# for i in range(vertices_raw.shape[0]):
#     vertices[int(vertices_raw[i,0])] = vertices_raw[i,1:]

# plt.scatter(vertices_raw[:,1],vertices_raw[:,2],marker='o',s=100)
for v in vertices_raw:
    angle = quat2euler(v[3:7]) if use_quat else v[2]
    rectangle_corner = np.array([v[0]-diag_len*np.cos(0.25*np.pi+angle),
                                v[1]-diag_len*np.sin(0.25*np.pi+angle)])
    rect = Rectangle((rectangle_corner[0],rectangle_corner[1]),
                robot_dims[0],robot_dims[1],
                edgecolor='purple',facecolor='purple',angle=180.*angle/np.pi)
    plt.gca().add_patch(rect)
    # Label vertices with their idx
    # for i in range(vertices_raw.shape[0]):
    #     plt.annotate(str(int(vertices_raw[i,0])),(vertices_raw[i,1],vertices_raw[i,2]))

#edges_fname = roadmap_dir+"edges.txt"
#edges = np.loadtxt(edges_fname,delimiter=",")
# '''
plots_dir = roadmap_dir + "points.png"
plt.savefig(plots_dir)
#plt.show()