from pickletools import optimize
import numpy as np 
import os
import subprocess
import yaml
import matplotlib.pyplot as plt 
from matplotlib.patches import Rectangle
from tqdm import tqdm
from PIL import Image
np.set_printoptions(suppress=True)

def f_sin(sim_time):
    return 6 * np.sin(0.33*sim_time)

def f_cos(sim_time):
    return 6 * np.cos(0.33*sim_time)

box_size = [1.0,1.0]
robot_dims = [0.508,0.430]
diag_len = 0.25 * np.sqrt(robot_dims[0]**2 + robot_dims[1]**2)
goal = [9.0,0.0]
goal_radius = 0.1
data_dir = os.environ["DIRTMP_PATH"]+"out/dynamic/prescience_test/"
step = 1
simulation_step = 0.1
num_trajs = 10
# num_trajs = 1

environment_file = os.environ["DIRTMP_PATH"]+"resources/input_files/environments/dynamic.yaml"

with open(environment_file, 'r') as stream:
    try:
        env_params = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

num_success = 0.0
plt.figure(figsize=(8,8))
for idx in tqdm(range(0,num_trajs)):

    traj = np.loadtxt(data_dir+"trajectory_"+str(idx)+".txt",delimiter=",")
    if sum(traj[:,-1]) == traj.shape[0]: num_success += 1
    continue_plotting = True
    first_collision_state = -1

    for i in tqdm(range(0,len(traj),step)):
        plt.xlim(-11,11)
        plt.ylim(-11,11)
        plt.gca().set_xticks([])
        plt.gca().set_yticks([])

        obstacles = env_params["environment"]["geometries"]
        for obstacle in obstacles:
            if not obstacle["dynamic"]:
                box_center = obstacle["config"]["position"][:2]
                box_dims = obstacle["collision_geometry"]["dims"][:2]
                rect = Rectangle((box_center[0]-box_dims[0]/2.0,box_center[1]-box_dims[1]/2.0),box_dims[0],box_dims[1],
                linewidth=1,edgecolor='r',facecolor='r')
                plt.gca().add_patch(rect)
            else:
                box_center = obstacle["config"]["position"][:2]
                if obstacle["name"] == "box1" or obstacle["name"] == "box3" or obstacle["name"] == "box5": box_center[1] = f_cos(i*simulation_step)
                else: box_center[1] = f_sin(i*simulation_step)
                box_dims = obstacle["collision_geometry"]["dims"][:2]
                rect = Rectangle((box_center[0]-box_dims[0]/2.0,box_center[1]-box_dims[1]/2.0),box_dims[0],box_dims[1],
                linewidth=1,edgecolor='r',facecolor='r')
                plt.gca().add_patch(rect)

        circle = plt.Circle((goal[0],goal[1]),goal_radius,color='green')
        plt.gca().add_patch(circle)

        plt.plot(traj[:,0],traj[:,1],color='black')

        if continue_plotting:
            # rectangle_corner = np.array([traj[i,0]-robot_dims[0]*0.5,traj[i,1]-robot_dims[1]*0.5])
            rectangle_corner = np.array([traj[i,0]-diag_len*np.cos(0.25*np.pi+traj[i,2]),
                                    traj[i,1]-diag_len*np.sin(0.25*np.pi+traj[i,2])])
            first_collision_state = i
            rect = Rectangle((rectangle_corner[0],rectangle_corner[1]),
                            robot_dims[0],robot_dims[1],
                            edgecolor='purple',facecolor='purple',angle=180.*traj[i,2]/np.pi)
            plt.gca().add_patch(rect)
        else:
            break
        continue_plotting = (continue_plotting and traj[max(0,i),-1] == 1)

        plt.text(6.0,9.0,"t =: "+f"{(i*simulation_step): .1f}"+"s",fontsize=10)
        plt.title("DIRT_Replan_NoPrescience")
        plt.savefig(data_dir+str(i)+".png",bbox_inches='tight')
        plt.clf()

    fnames = [Image.open(data_dir+str(i)+".png") for i in range(0,first_collision_state+1,step)]
    fnames[0].save(data_dir+'output_'+str(idx)+'.gif',format='GIF',append_images=fnames[1:],
    save_all=True,duration=50,optimize=True)
    subprocess.call("cd " + data_dir + " && rm -rf *.png",shell=True)