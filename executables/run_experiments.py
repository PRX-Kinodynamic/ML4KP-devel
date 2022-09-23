import yaml
import os 
import subprocess
import time

class Fname:
    def __init__(self,fname):
        self.fname = fname
    
def fname_constructor(loader,node):
    return Fname(node.value)

yaml.SafeLoader.add_constructor("!file",fname_constructor)

planners = ["dirt"]
systems = ["omnirobot_mecanum_FO", "fo_treaded_vehicle", "unicycle_fo_v0", "treaded_vehicle"]
systems = ["treaded_vehicle"]

for planner in planners:
    original_fname = os.environ["DIRTMP_PATH"] + "resources/input_files/examples/benchmark/run_" + planner + ".yaml"
    planner_params = {}
    with open(original_fname, 'r') as stream:
        try:
            planner_params = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    del planner_params["plant"]
    planner_params["goal_region_radius"] = 0.5
    for system in systems:
        if system == "omnirobot_mecanum_FO":
            planner_params["planning_time"] = 10
            planner_params["environment"] = "environments/BARN.yaml"
        elif system == "fo_treaded_vehicle":
            planner_params["planning_time"] = 60
            planner_params["environment"] = "environments/warehouse_benchmr.yaml"
        elif system == "unicycle_fo_v0":
            planner_params["planning_time"] = 60
            planner_params["environment"] = "environments/kmp_benchmark/unicycle_fo/kink_0.yaml"
            planner_params["goal_region_radius"] = 0.1
        elif system == "treaded_vehicle":
            planner_params["planning_time"] = 60
            planner_params["environment"] = "environments/warehouse_benchmr.yaml"
            planner_params["goal_region_radius"] = 1.0
        elif system == "unicycle_so":
            planner_params["planning_time"] = 300
            planner_params["environment"] = "environments/kmp_benchmark/unicycle_so/kink_0.yaml"
            planner_params["goal_region_radius"] = 0.2
        planner_params["output_dir"] = "bench/" + planner + "_" + system
        with open (os.environ["DIRTMP_PATH"]+"resources/input_files/examples/test.yaml","w") as f:
            yaml.dump(planner_params,f, sort_keys=False)
            f.write("plant: !file \"plants/" + system + ".yaml\"")
        cmd = [os.environ["DIRTMP_PATH"] + "bin/examples/benchmark/run_" + planner, "examples/test.yaml"]
        popen = subprocess.Popen(cmd)
        time.sleep(1)