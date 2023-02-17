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
    def __init__(self,mode):
        self.original_yaml_path = os.environ["DIRTMP_PATH"] + "resources/input_files/examples/mujoco/mushr_trajectory.yaml"
        self.planner_params = {}
        with open(self.original_yaml_path, 'r') as f:
            try:
                self.planner_params = yaml.safe_load(f)
            except yaml.YAMLError as exc:
                print(exc)
        del self.planner_params["plant"]
        del self.planner_params["learned_controller"]

    def run(self, start, goal, mode, name, id=0):
        self.planner_params["start_state"] = start.tolist()
        self.planner_params["goal_state"] = goal.tolist()
        self.planner_params["output_dir"] = "ablation/"+mode+"/"+str(id)+"/"  
        self.planner_params["planner_name"] = name
        if name == "random":
            self.planner_params["use_rlg"] = False 
        elif name == "rlg":
            self.planner_params["use_rlg"] = True

        # test_yaml_fname = "test_"+mode+"_"+name+"_"+str(id)+".yaml"
        test_yaml_fname = "test.yaml"
        with open(os.environ["DIRTMP_PATH"]+'resources/input_files/examples/'+test_yaml_fname,'w') as f:
            yaml.safe_dump(self.planner_params, f, default_flow_style=False)
            f.write("plant: !file \"plants/mushr.yaml\"\n")
            f.write("learned_controller: !file \"networks/mushr.yaml\"\n")

       
        cmd = [os.environ["DIRTMP_PATH"]+"bin/examples/mujoco/run_rlg_with_stats","examples/"+test_yaml_fname]
        popen = subprocess.Popen(cmd)
        time.sleep(1)
        # popen.wait()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', action='store_true')
    parser.add_argument('--mode', type=str, default="indoors")
    parser.add_argument('--name', type=str, default='random')

    args = parser.parse_args()
    run = args.run
    mode = args.mode
    name = args.name

    print("run tests with settings: run=",run,", mode=",mode,", name=",name)
    if run:
        sr = ScriptRunner(mode)
        landmarks = np.loadtxt(os.environ["DIRTMP_PATH"]+"executables/indoors_goals.txt",delimiter=",")
        counter = 0
        for i in tqdm(range(landmarks.shape[0])):
            print("run test ",counter)
            s = landmarks[i,:5]
            g = landmarks[i,5:]
            sr.run(s,g,mode,name,counter)
            counter += 1