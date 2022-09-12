import yaml
import os
import subprocess 
import numpy as np 

class Fname:
    def __init__(self,fname):
        self.fname = fname
    
def fname_constructor(loader,node):
    return Fname(node.value)

yaml.SafeLoader.add_constructor("!file",fname_constructor)

class ScriptRunner:
    def __init__(self):
        self.original_yaml_path = os.environ["DIRTMP_PATH"] + "resources/input_files/local_goal/annotate_segway.yaml"
        self.planner_params = {}
        with open(self.original_yaml_path, 'r') as f:
            try:
                self.planner_params = yaml.safe_load(f)
            except yaml.YAMLError as exc:
                print(exc)
        del self.planner_params["plant"]
        del self.planner_params["learned_controller"]

    def run(self, start, goal,id=0):
        self.planner_params["start_state"] = start.tolist()
        self.planner_params["goal_state"] = goal.tolist()
        self.planner_params["output_id"] = id

        with open(os.environ["DIRTMP_PATH"]+'resources/input_files/examples/test.yaml','w') as f:
            yaml.safe_dump(self.planner_params, f, default_flow_style=False)
            f.write("plant: !file \"plants/segway.yaml\"\n")
            f.write("learned_controller: !file \"networks/segway.yaml\"\n")

        cmd = [os.environ["DIRTMP_PATH"]+"bin/executables/local_goal/controller_trajectory_bullet","examples/test.yaml"]
        popen = subprocess.Popen(cmd, stdout=subprocess.PIPE)
        popen.wait()

if __name__ == "__main__":
    sr = ScriptRunner()
    sr.run(np.array([0,0,0,0,0,0,0]),np.array([1,1,1,1,1,1,1]))
