import yaml
import os
import subprocess 
import numpy as np 
import time
import argparse
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from tqdm import tqdm

class Fname:
    def __init__(self,fname):
        self.fname = fname
    
def fname_constructor(loader,node):
    return Fname(node.value)

yaml.SafeLoader.add_constructor("!file",fname_constructor)

class ScriptRunner:
    def __init__(self):
        self.original_yaml_path = os.environ["DIRTMP_PATH"] + "resources/input_files/local_goal/annotate_treaded.yaml"
        self.planner_params = {}
        with open(self.original_yaml_path, 'r') as f:
            try:
                self.planner_params = yaml.safe_load(f)
            except yaml.YAMLError as exc:
                print(exc)
        del self.planner_params["plant"]
        del self.planner_params["learned_controller"]
        del self.planner_params["termination_classifier"]

    def run(self, start, goal, mode, id=0):
        self.planner_params["start_state"] = start.tolist()
        self.planner_params["goal_state"] = goal.tolist()
        self.planner_params["output_dir"] = "1026/"+mode+"/"+str(id)+"/"

        with open(os.environ["DIRTMP_PATH"]+'resources/input_files/examples/test.yaml','w') as f:
            yaml.safe_dump(self.planner_params, f, default_flow_style=False)
            f.write("plant: !file \"plants/treaded_vehicle.yaml\"\n")
            f.write("learned_controller: !file \"networks/treaded_vehicle_controller.yaml\"\n")
            f.write("termination_classifier: !file \"networks/svm_classifier.yaml\"\n")

        cmd = [os.environ["DIRTMP_PATH"]+"bin/executables/local_goal/landmark_roadmap","examples/test.yaml"]
        popen = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        popen.wait()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--mode', type=str, default="easy")

    args = parser.parse_args()
    run = args.run
    mode = args.mode

    if run:
        sr = ScriptRunner()
        landmarks = np.loadtxt(os.environ["DIRTMP_PATH"]+"out/landmarks.txt",delimiter=",")
        counter = 0
        for i in tqdm(range(landmarks.shape[0])):
            for j in range(i+1,landmarks.shape[0]):
                if i != j and np.linalg.norm(landmarks[i]-landmarks[j]) >= 5.0:
                    sr.run(landmarks[i],landmarks[j],mode,counter)
                    counter += 1
                    time.sleep(0.1)
    else:
        environment_file = os.environ["DIRTMP_PATH"]+"resources/input_files/environments/landmark.yaml"
        with open(environment_file, 'r') as f:
            try:
                env_params = yaml.safe_load(f)
            except yaml.YAMLError as exc:
                print(exc)
        
        landmarks = np.loadtxt(os.environ["DIRTMP_PATH"]+"out/landmarks.txt",delimiter=",")

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

        plt.scatter(landmarks[:,0],landmarks[:,1],c="b")

        out_dir = os.environ["DIRTMP_PATH"]+"out/1026/"+mode+"/"
        
        for f in os.listdir(out_dir):
            if os.path.isdir(out_dir+f):
                with open(out_dir+f+"/traj.txt") as f:
                    path = np.loadtxt(f,delimiter=",")
                    if path.shape[0] != 0:
                        plt.plot(path[:,0],path[:,1],c="black")
        
        plt.show()

        for i, f in enumerate(os.listdir(out_dir)):
            if os.path.isdir(out_dir+f):
                with open(out_dir+f+"/solution.txt") as f:
                    path = np.loadtxt(f,delimiter=",")
                    print(i,",",path[0],",",path[1])

        
        

        

