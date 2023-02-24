import numpy as np
import matplotlib.pyplot as plt
import os
import yaml
import math
from xml.dom import minidom
from tqdm import tqdm
import matplotlib
import argparse
from matplotlib.patches import Arrow, Circle, Rectangle


def quat2euler(quat):
    # Function that converts a quaternion [w,x,y,z] to euler angles [roll,pitch,yaw]
    # Source: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    # Assumes that the quaternion is normalized
    w,x,y,z = quat
    roll = np.arctan2(2*(w*x+y*z),1-2*(x**2+y**2))
    pitch = np.arcsin(2*(w*y-z*x))
    yaw = np.arctan2(2*(w*z+x*y),1-2*(y**2+z**2))
    return yaw

parser = argparse.ArgumentParser()
parser.add_argument('--target', type=str, default="roadmap_tree_0")
parser.add_argument('--dir', type=str, default="out/ablation/indoors/")
parser.add_argument('--g', action="store_true")
args = parser.parse_args()
target = args.target
dir = args.dir
if not args.g:
    dir = os.environ['DIRTMP_PATH'] + dir

fname = dir + target+"/"
env_fname_prefix = os.environ['DIRTMP_PATH'] + "resources/models/mujoco/"
env_fname = "indoor.xml"

goal_state = np.array([-4.0,-5.0])
goal_radius = 0.5

plt.figure(figsize=(8, 8))
plt.xlim(-10,10)
plt.ylim(-10,10)


print('matplotlib: {}'.format(matplotlib.__version__))

if env_fname.endswith(".yaml"):
    with open(env_fname_prefix + env_fname) as f:
        try:
            data = yaml.safe_load(f)
        except yaml.YAMLError as exc:
            print(exc)
        
    obstacles = data["environment"]["geometries"]
    for obstacle in obstacles:
        if obstacle["collision_geometry"]["type"] == "box":
            box_center = obstacle["config"]["position"]
            box_dims   = obstacle["collision_geometry"]["dims"]
            rect = Rectangle((box_center[0]-box_dims[0]*0.5, box_center[1]-box_dims[1]*0.5), box_dims[0], box_dims[1], color='red')
            plt.gca().add_patch(rect)
        else:
            print("Only box obstacles are supported")
            exit()

elif env_fname.endswith(".xml"):
    xmldoc = minidom.parse(env_fname_prefix + env_fname)
    # Get all <body> <geom> tags
    body_list = xmldoc.getElementsByTagName('body')
    for body in body_list:
        geom_list = body.getElementsByTagName('geom')
        for geom in geom_list:
            if geom.attributes['type'].value == "box":
                box_orient = 0
                # Check if box has euler attribute
                if 'euler' in geom.attributes:
                    box_orient = math.degrees(float(geom.attributes['euler'].value.split()[2]))
                box_center = [float(x) for x in geom.attributes['pos'].value.split()]
                box_dims   = [float(x) for x in geom.attributes['size'].value.split()]
                rect = Rectangle((box_center[0]-box_dims[0], box_center[1]-box_dims[1]), 2*box_dims[0], 2*box_dims[1], angle = box_orient, rotation_point= 'center', color='red')
                plt.gca().add_patch(rect)
            else:
                print("Only box obstacles are supported")
                exit()
else:
    pass

circle = Circle(goal_state, goal_radius, color='green')
plt.gca().add_patch(circle)

for f in tqdm(os.listdir(fname)):
    if f.endswith(".txt"):
        data = np.loadtxt(fname+f,delimiter=',')
        if f.startswith("tree"):
            plt.plot(data[:,0], data[:,1], color='black',linewidth=0.5)
        if f.startswith("solution") and data.shape[0] > 1:
            plt.plot(data[:,0], data[:,1], color='red', linewidth=2)
try:
    path_fname = dir + "path_roadmap.txt"
    path = np.loadtxt(path_fname,delimiter=' ')

    x = []
    y = []
    markers = []
    for v in path:
        x.append(v[0])
        y.append(v[1])

    plt.plot(x,y,'g')

    for v in path:
        angle = quat2euler(v[3:7])
        arr = Arrow(v[0],v[1],
                    0.5*np.cos(angle),0.5*np.sin(angle),zorder=2.5)
        plt.gca().add_patch(arr)
except:
    None

plots_dir = dir+target+".png"
plt.savefig(plots_dir)
print("saved: "+target)