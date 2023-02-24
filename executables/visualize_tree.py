import numpy as np
import matplotlib.pyplot as plt
import os
import yaml
import math
from xml.dom import minidom
from tqdm import tqdm
import matplotlib
import argparse
from matplotlib.patches import Circle, Rectangle


parser = argparse.ArgumentParser()
parser.add_argument('--target', type=str, default="roadmap_tree_0")
args = parser.parse_args()
target = args.target

fname = os.environ['DIRTMP_PATH'] + "out/ablation/indoors/"+ target+"/"
env_fname_prefix = os.environ['DIRTMP_PATH'] + "resources/models/mujoco/"
env_fname = "indoor.xml"
goal_state = np.array([-9.,-5.])
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

plots_dir = os.environ["DIRTMP_PATH"]+"out/"+target+".png"
plt.savefig(plots_dir)