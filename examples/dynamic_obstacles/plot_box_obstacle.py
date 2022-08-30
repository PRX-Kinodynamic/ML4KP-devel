from pickletools import optimize
from time import time
import numpy as np 
import os
import subprocess
import yaml
import matplotlib.pyplot as plt 
import pandas as pd
from matplotlib.patches import Rectangle
from tqdm import tqdm
from PIL import Image
np.set_printoptions(suppress=True)

class Obstacle:
    def __init__(self,init_pos,orn,vel,name):
        self.init_pos = init_pos
        self.orn = orn
        self.vel = vel
        self.box_dims = [-1,-1]
        self.last_reset_time = 0
        self.diag_len = 0
        self.name = name
        self.poses = []
    
    def set_box_dims(self,box_dims):
        self.box_dims = box_dims
        self.diag_len = 0.25 * np.sqrt(box_dims[0]**2 + box_dims[1]**2) 
    
    def set_f(self,world_info):
        self.poses = np.vstack([world_info[self.name+"_x"],world_info[self.name+"_y"]])

    def f(self,time_idx):
        return self.poses[:,time_idx]

robot_dims = [0.508,0.430]
diag_len = 0.25 * np.sqrt(robot_dims[0]**2 + robot_dims[1]**2)
goal = [9.0,0.0]
goal_radius = 0.1
data_dir = os.environ["DIRTMP_PATH"]+"out/dynamic/evaluation/greedy2/8/"
simulation_step = 0.01
step = int(0.1/simulation_step)
num_trajs = 10

environment_file = os.environ["DIRTMP_PATH"]+"resources/input_files/environments/test_dynamic_box/box_8.yaml"

with open(environment_file, 'r') as stream:
    try:
        env_params = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

obstacles = []
obstacles_yml = env_params["environment"]["dynamic_geometries"]
for obstacle in obstacles_yml:
    obstacles.append(Obstacle(obstacle["config"]["position"],obstacle["config"]["rotation"],obstacle["multiplier"],obstacle["name"]))
    obstacles[0].set_box_dims(obstacle["collision_geometry"]["dims"][:2])

plt.figure(figsize=(8,8))
for idx in tqdm(range(3)):
# for idx in tqdm(range(0,num_trajs)):
    traj = np.loadtxt(data_dir+"trajectory_"+str(idx)+".txt",delimiter=",")
    obs_infos = pd.read_csv(data_dir+"infos_"+str(idx)+".txt",delimiter=",")
    waypts = []
    if os.path.isfile(data_dir+"waypts_"+str(idx)+".txt"):
        waypts = np.genfromtxt(data_dir+"waypts_"+str(idx)+".txt",delimiter=",",usecols=[0,1,2,3,4])
    continue_plotting = True
    first_collision_state = -1

    for obstacle in obstacles:
        obstacle.set_f(obs_infos)

    for i in tqdm(range(0,len(traj),step)):
        plt.xlim(-11,11)
        plt.ylim(-11,11)
        plt.gca().set_xticks([])
        plt.gca().set_yticks([])
        if len(waypts) > 0: 
            plt.scatter(waypts[:,0],waypts[:,1],color='green',marker='.')

        obstacles_yml = env_params["environment"]["geometries"]
        for obstacle in obstacles_yml:
            if not obstacle["dynamic"]:
                box_center = obstacle["config"]["position"][:2]
                box_dims = obstacle["collision_geometry"]["dims"][:2]
                rect = Rectangle((box_center[0]-box_dims[0]/2.0,box_center[1]-box_dims[1]/2.0),box_dims[0],box_dims[1],
                linewidth=1,edgecolor='r',facecolor='r')
                plt.gca().add_patch(rect)
        
        for obstacle in obstacles:
            box_center = obstacle.f(i)
            plt.text(box_center[0],box_center[1],
                    obstacle.name,fontsize=10)
            box_dims = obstacle.box_dims
            rect = Rectangle((box_center[0]-box_dims[0]/2.0,box_center[1]-box_dims[1]/2.0),box_dims[0],box_dims[1],
                linewidth=1,edgecolor='r',facecolor='r',angle=180. * obstacle.orn / np.pi)
            plt.gca().add_patch(rect)

        circle = plt.Circle((goal[0],goal[1]),goal_radius,color='green')
        plt.gca().add_patch(circle)

        plt.plot(traj[:,0],traj[:,1],color='black')

        if continue_plotting:
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
        plt.title("DIRT_NoReplan_Prescience")
        plt.savefig(data_dir+str(i)+".png",bbox_inches='tight')
        plt.clf()

        vels = np.array([np.linalg.norm(traj[i,-2:]) for i in range(0,traj.shape[0])])

    fnames = [Image.open(data_dir+str(i)+".png") for i in range(0,first_collision_state+1,step)]
    fnames[0].save(data_dir+'output_'+str(idx)+'.gif',format='GIF',append_images=fnames[1:],
    save_all=True,duration=50,optimize=True)
    subprocess.call("cd " + data_dir + " && rm -rf *.png",shell=True)
