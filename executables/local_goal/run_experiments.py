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
        if mode == "treaded":
            self.original_yaml_path = os.environ["DIRTMP_PATH"] + "resources/input_files/local_goal/annotate_treaded.yaml"
        else:
            self.original_yaml_path = os.environ["DIRTMP_PATH"] + "resources/input_files/local_goal/car_like.yaml"
        self.planner_params = {}
        with open(self.original_yaml_path, 'r') as f:
            try:
                self.planner_params = yaml.safe_load(f)
            except yaml.YAMLError as exc:
                print(exc)
        del self.planner_params["plant"]
        del self.planner_params["learned_controller"]
        del self.planner_params["termination_classifier"]

    def run(self, start, goal, mode, name, id=0):
        self.planner_params["start_state"] = start.tolist()
        self.planner_params["goal_state"] = goal.tolist()
        # Todo: Clean this up
        self.planner_params["output_dir"] = "rooms_" + mode + "/planning/"+str(id)+"/"
        # self.planner_params["output_dir"] = "warehouse_" + mode + "/planning/"+str(id)+"/"
        # self.planner_params["output_dir"] = "indoor_" + mode + "/planning/"+str(id)+"/"
            
        if name == "random":
            self.planner_params["random_local_goal"] = False 
            self.planner_params["planner_name"] = "Random"
            cmd = [os.environ["DIRTMP_PATH"]+"bin/executables/local_goal/random_rlg","examples/"+test_yaml_fname]
        elif name == "rlg":
            self.planner_params["random_local_goal"] = True
            self.planner_params["planner_name"] = "RLG"
            cmd = [os.environ["DIRTMP_PATH"]+"bin/executables/local_goal/random_rlg","examples/"+test_yaml_fname]
        elif name == "greedy":
            self.planner_params["planner_name"] = "GreedyRoGuE"
            cmd = [os.environ["DIRTMP_PATH"]+"bin/executables/local_goal/rogue_greedy","examples/"+test_yaml_fname]
        elif name == "shortcut":
            self.planner_params["planner_name"] = "ShortcutRoGuE"
            cmd = [os.environ["DIRTMP_PATH"]+"bin/executables/local_goal/rogue_shortcut","examples/"+test_yaml_fname]

        # test_yaml_fname = "test_"+mode+"_"+name+"_"+str(id)+".yaml"
        test_yaml_fname = "test.yaml"
        with open(os.environ["DIRTMP_PATH"]+'resources/input_files/examples/'+test_yaml_fname,'w') as f:
            yaml.safe_dump(self.planner_params, f, default_flow_style=False)
            if mode == "treaded":
                f.write("plant: !file \"plants/treaded_vehicle.yaml\"\n")
                f.write("learned_controller: !file \"networks/treaded_vehicle_controller.yaml\"\n")
            else:
                f.write("plant: !file \"plants/car_like.yaml\"\n")
                f.write("learned_controller: !file \"networks/car_like_controller.yaml\"\n")
            f.write("termination_classifier: !file \"networks/svm_classifier.yaml\"\n")

        popen = subprocess.Popen(cmd)
        time.sleep(1)
        # popen.wait()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', type=str, default="car")
    parser.add_argument('--name', type=str, default='roadmap')

    args = parser.parse_args()
    run = args.run
    mode = args.mode
    name = args.name

    sr = ScriptRunner(mode)
    landmarks = np.loadtxt(os.environ['DIRTMP_PATH']+"out/rooms_car/landmarks.txt",delimiter=",")
    # landmarks = np.loadtxt(os.environ["DIRTMP_PATH"]+"out/warehouse_car/landmarks.txt",delimiter=",")
    # landmarks = np.loadtxt(os.environ["DIRTMP_PATH"]+"out/indoor_car/landmarks.txt",delimiter=",")
    counter = 0
    for i in tqdm(range(landmarks.shape[0])):
        s = landmarks[i,:5]
        g = landmarks[i,5:]
        sr.run(s,g,mode,name,counter)
        counter += 1
