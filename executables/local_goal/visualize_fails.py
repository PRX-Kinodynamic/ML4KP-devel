import numpy as np 
import matplotlib.pyplot as plt 
import matplotlib.patches as patches
import os
import yaml
from tqdm import tqdm

data_dir = os.environ["DIRTMP_PATH"]+"out/smoothen_classify/"
environment_file = os.environ["DIRTMP_PATH"]+"resources/input_files/environments/narrow.yaml"

with open(environment_file, 'r') as stream:
    try:
        env_params = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

plt.figure(figsize=(8,8))

obstacles = env_params["environment"]["geometries"]
for obstacle in obstacles:
    box_center = obstacle["config"]["position"][:2]
    box_dims = obstacle["collision_geometry"]["dims"][:2]
    rect = patches.Rectangle((box_center[0]-box_dims[0]/2.0,box_center[1]-box_dims[1]/2.0),box_dims[0],box_dims[1],linewidth=1,edgecolor='r',facecolor='r')
    plt.gca().add_patch(rect)

for i, fname in tqdm(enumerate(os.listdir(data_dir))):
    if not fname.endswith('.txt'): continue
    if not (fname[0] == "a" or fname[0] == "d"): continue
    traj = np.loadtxt(data_dir+fname,delimiter=",")
    
    plt.scatter(traj[0,0],traj[0,1],c='g',s=50,label='Start')
    plt.scatter(traj[-1,0],traj[-1,1],c='r',s=50,label='Goal',marker='x')
    plt.plot(traj[:,0],traj[:,1],linewidth=1,color='black',label='Controller')

plt.xlim(-15,15)
plt.ylim(-15,15)
# plt.legend(loc='best')
plt.show()
# plt.savefig(data_dir+"viz_"+str(index)+".png")
plt.close()