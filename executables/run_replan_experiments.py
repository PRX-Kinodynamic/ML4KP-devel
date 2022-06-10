import yaml
import os
import subprocess
import time
from tqdm import tqdm

class Fname:
    def __init__(self,fname):
        self.fname = fname
    
def fname_constructor(loader,node):
    return Fname(node.value)

yaml.SafeLoader.add_constructor("!file",fname_constructor)

original_yaml_path = os.environ["DIRTMP_PATH"]+"resources/input_files/examples/dynamic_obstacles/replan_test.yaml"

planner_params = {}
with open(original_yaml_path, 'r') as stream:
    try:
        planner_params = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

del planner_params["plant"]
planner_params["num_trials"] = 30

planning_times = [0.1, 0.2, 0.5]
buffer_times   = [0.1, 0.2, 0.5]
horizons = [0.5, 1.0, 2.0]

for pt in tqdm(planning_times):
    for bt in buffer_times:
        for h in horizons:
            if pt + bt > h: continue
            planner_params["planning_time"] = pt
            planner_params["buffer_time"]   = bt
            planner_params["horizon"]       = h

            planner_params["output_dir"] = "dynamic/06_10/" + str(pt) + "_" + str(bt) + "_" + str(h)

            with open(os.environ["DIRTMP_PATH"]+'resources/input_files/examples/test.yaml', 'w') as f:
                yaml.safe_dump(planner_params, f, sort_keys=False)
                f.write("plant: !file \"plants/treaded_vehicle.yaml\"\n")
            
            cmd = [os.environ["DIRTMP_PATH"]+"bin/examples/dynamic_obstacles/replan_test","examples/test.yaml"]
            popen = subprocess.Popen(cmd, stdout=subprocess.PIPE)
            popen.wait()