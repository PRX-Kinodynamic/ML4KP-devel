import numpy as np 
import os
import yaml
import matplotlib.pyplot as plt 
from matplotlib.patches import Rectangle
from tqdm import tqdm
from PIL import Image

def f(sim_time):
    return 10 * np.sin(sim_time)

box_size = [1.0,1.0]
robot_dims = [0.508,0.430]
goal = [9.0,0.0]
goal_radius = 0.1
data_dir = os.environ["DIRTMP_PATH"]+"out/dynamic/"
simulation_step = 0.1

environment_file = os.environ["DIRTMP_PATH"]+"resources/input_files/environments/dynamic.yaml"

with open(environment_file, 'r') as stream:
    try:
        env_params = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

traj = np.loadtxt(data_dir+"trajectory.txt",delimiter=",")
continue_plotting = True
first_collision_state = -1

plt.figure(figsize=(8,8))
for i in tqdm(range(0,len(traj))):
    plt.xlim(-11,11)
    plt.ylim(-11,11)
    plt.gca().set_xticks([])
    plt.gca().set_yticks([])

    obstacles = env_params["environment"]["geometries"]
    for obstacle in obstacles:
        if obstacle["dynamic"]: continue
        box_center = obstacle["config"]["position"][:2]
        box_dims = obstacle["collision_geometry"]["dims"][:2]
        rect = Rectangle((box_center[0]-box_dims[0]/2.0,box_center[1]-box_dims[1]/2.0),box_dims[0],box_dims[1],
        linewidth=1,edgecolor='r',facecolor='r')
        plt.gca().add_patch(rect)

    circle = plt.Circle((goal[0],goal[1]),goal_radius,color='green')
    plt.gca().add_patch(circle)

    plt.plot(traj[:,0],traj[:,1],color='black')

    if continue_plotting:
        rectangle_corner = np.array([traj[i,0]-robot_dims[0]*0.5,traj[i,1]-robot_dims[1]*0.5])
        first_collision_state = i
        rect = Rectangle((rectangle_corner[0],rectangle_corner[1]),
                        robot_dims[0],robot_dims[1],
                        edgecolor='purple',facecolor='purple',angle=180*traj[i,2]/np.pi)
        plt.gca().add_patch(rect)
    else:
        break
    continue_plotting = (continue_plotting and traj[max(0,i-1),-1] == 1)

    rectangle_corner = np.array([0- box_size[0]*0.5,f(i*simulation_step)- box_size[1]*0.5])
    rect = Rectangle((rectangle_corner[0],rectangle_corner[1]),
                     box_size[0], box_size[1],
                     edgecolor='red',facecolor='red')
    plt.gca().add_patch(rect)

    plt.title("DIRT_NoReplan_NoPrescience")
    plt.savefig(data_dir+str(i)+".png")
    plt.clf()


fnames = [Image.open(data_dir+str(i)+".png") for i in range(first_collision_state)]
fnames[0].save(data_dir+'output.gif',format='GIF',append_images=fnames[1:],
save_all=True,duration=10)