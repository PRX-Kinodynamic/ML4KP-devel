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
del planner_params["waypoint_predictor"]
planner_params["num_trials"] = 10

planning_time = 2.0
horizon = 5.0

for i in range(10):
    planner_params["planning_time"] = planning_time
    planner_params["horizon"] = horizon

    planner_params["environment"] = "environments/test_dynamic_box/box_"+str(i)+".yaml"
    planner_params["output_dir"] = "dynamic/evaluation/greedy/"+str(i)

    with open(os.environ["DIRTMP_PATH"]+'resources/input_files/examples/test.yaml', 'w') as f:
        yaml.safe_dump(planner_params, f, sort_keys=False)
        f.write("plant: !file \"plants/treaded_vehicle.yaml\"\n")
        f.write("waypoint_predictor: !file \"networks/treaded_waypoint.yaml\"\n")
    
    cmd = [os.environ["DIRTMP_PATH"]+"bin/examples/dynamic_obstacles/replan_test","examples/test.yaml"]
    popen = subprocess.Popen(cmd)
    time.sleep(1)


# planning_times = [2.0]
# horizons = [5.0, 10.0]

# for pt in tqdm(planning_times):
#     for h in horizons:
#         if pt < h:
#             planner_params["planning_time"] = pt
#             planner_params["horizon"]       = h
#             planner_params["max_replanning_cycles"] = int(100/pt)

#             planner_params["output_dir"] = "dynamic/prescience/" + str(pt) + "_" + str(h)

#             with open(os.environ["DIRTMP_PATH"]+'resources/input_files/examples/test.yaml', 'w') as f:
#                 yaml.safe_dump(planner_params, f, sort_keys=False)
#                 f.write("plant: !file \"plants/treaded_vehicle.yaml\"\n")
            
#             cmd = [os.environ["DIRTMP_PATH"]+"bin/examples/dynamic_obstacles/replan_test","examples/test.yaml"]
#             popen = subprocess.Popen(cmd)
#             time.sleep(1)

print("Done!")