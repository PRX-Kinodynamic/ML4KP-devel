from random import random
import yaml 
import os 
import numpy as np 
import argparse
import subprocess
import time


def write_yaml(idx,test_seed=False,type="barn"):
    if test_seed:
        np.random.seed(idx + 210896)
        fname_prefix = os.environ["DIRTMP_PATH"]+"resources/input_files/environments/test_dynamic_box/"
    else:
        np.random.seed(idx)
        fname_prefix = os.environ["DIRTMP_PATH"]+"resources/input_files/environments/train_dynamic_box/"
    fname = fname_prefix+"box_"+str(idx)+".yaml"

    yaml_to_dump = {}
    yaml_to_dump["environment"] = {}
    yaml_to_dump["environment"]["type"] = "obstacle"
    yaml_to_dump["environment"]["geometries"] = []
    yaml_to_dump["environment"]["dynamic_geometries"] = []

    upper_row = {
        "name": "upper",
        "dynamic": False,
        "collision_geometry": {
            "type": "box",
            "dims": [21,0.5,0.2],
            "material": "red"
        },
        "config": {
            "position": [0,10.25,0],
            "orientation": [0,0,0,1]
        }
    }

    lower_row = {
        "name": "lower",
        "dynamic": False,
        "collision_geometry": {
            "type": "box",
            "dims": [21,0.5,0.2],
            "material": "red"
        },
        "config": {
            "position": [0,-10.25,0],
            "orientation": [0,0,0,1]
        }
    }

    right_row = {
        "name": "right",
        "dynamic": False,
        "collision_geometry": {
            "type": "box",
            "dims": [0.5,21,0.2],
            "material": "red"
        },
        "config": {
            "position": [10.25,0,0],
            "orientation": [0,0,0,1]
        }
    }

    left_row = {
        "name": "left",
        "dynamic": False,
        "collision_geometry": {
            "type": "box",
            "dims": [0.5,21,0.2],
            "material": "red"
        },
        "config": {
            "position": [-10.25,0,0],
            "orientation": [0,0,0,1]
        }
    }

    yaml_to_dump["environment"]["geometries"].append(upper_row)
    yaml_to_dump["environment"]["geometries"].append(lower_row)
    yaml_to_dump["environment"]["geometries"].append(right_row)
    yaml_to_dump["environment"]["geometries"].append(left_row)

    if type == "barn":
        yaml_to_dump["environment"]["dynamic_geometries"] = generate_barn_obstacles()
    elif type == "swap":
        yaml_to_dump["environment"]["dynamic_geometries"] = generate_swap_obstacles()

    with open(fname, 'w') as outfile:
        yaml.safe_dump(yaml_to_dump, outfile, default_flow_style=None)

def generate_barn_obstacles():
    rows = []

    # num_boxes = np.random.randint(10,15)
    num_boxes = 5
    for i in range(0,num_boxes):
        row = {}
        row["name"] = "box_"+str(i)
        row["position_function"] = "triangle"
        row["multiplier"] = np.random.uniform(1.0,1.5)
        row["collision_geometry"] = {}
        row["collision_geometry"]["type"] = "box"
        row["collision_geometry"]["material"] = "red"
        # row["collision_geometry"]["dims"] = [np.random.uniform(0.1,0.5),np.random.uniform(0.1,0.5),0.2]
        row["collision_geometry"]["dims"] = [0.5,0.5,0.2]
        row["config"] = {}
        row["config"]["position"] = [np.random.uniform(-8.0,8.0),np.random.uniform(-8.0,8.0),0.0]
        row["config"]["rotation"] = np.random.uniform(-0.5*np.pi,0.5*np.pi)
        rows.append(row)

    return rows

def generate_swap_obstacles():
    rows = []

    num_boxes = 2
    # Create num_boxes bins between -7.5 and 7.5
    bins = np.linspace(-7.5,7.5,num_boxes+1)

    for i in range(num_boxes):
        row_x = np.random.uniform(bins[i],bins[i+1])
        row_y = np.random.uniform(-7.5,7.5)
        row = {}
        row["name"] = "box_"+str(i)
        row["position_function"] = "oscillate"
        row["multiplier"] = np.random.uniform(1.0,1.5)
        row["collision_geometry"] = {}
        row["collision_geometry"]["type"] = "box"
        row["collision_geometry"]["material"] = "red"
        row["collision_geometry"]["dims"] = [1.5,1.5,0.2]
        row["config"] = {}
        row["config"]["position"] = [row_x,row_y,0.0]
        row["config"]["rotation"] = float(np.random.choice([-np.pi/2,np.pi/2]))
        rows.append(row)
    
    return rows

class Fname:
    def __init__(self,fname):
        self.fname = fname
    
def fname_constructor(loader,node):
    return Fname(node.value)

yaml.SafeLoader.add_constructor("!file",fname_constructor)

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_files", type=int, default=10)
    parser.add_argument("--num_trials", type=int, default=10)
    parser.add_argument("--run_planner", action="store_true")
    parser.add_argument("--test", action="store_true")
    parser.add_argument("--type", type=str, default="barn")
        
    args  = parser.parse_args()

    for i in range(0,args.num_files):
        write_yaml(i,args.test,args.type)

    if args.run_planner:
        original_yaml_path = os.environ["DIRTMP_PATH"]+"resources/input_files/examples/dynamic_obstacles/dynamic_obstacles_test.yaml"

        planner_params = {}
        with open(original_yaml_path, 'r') as stream:
            try:
                planner_params = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        del planner_params["plant"]
        planner_params["num_trials"] = 1

        for i in range(0,args.num_files):
            for j in range(0,args.num_trials):
                planner_params["sample_start_goal"] = True
                # planner_params["sample_start_goal"] = False
                planner_params["random_seed"] = i + j + np.random.randint(101193,210896)
                if not args.test:
                    planner_params["environment"] = "environments/train_dynamic_box/box_"+str(i)+".yaml"
                    planner_params["output_dir"] = "dynamic/prescience_data/" + str(i) + "_" + str(j)
                else:
                    planner_params["environment"] = "environments/test_dynamic_box/box_"+str(i)+".yaml"
                    planner_params["output_dir"] = "dynamic/prescience_data_test/" + str(i) + "_" + str(j)

                with open(os.environ["DIRTMP_PATH"]+'resources/input_files/examples/test.yaml', 'w') as f:
                    yaml.safe_dump(planner_params, f, sort_keys=False)
                    f.write("plant: !file \"plants/treaded_vehicle.yaml\"\n")
                
                cmd = [os.environ["DIRTMP_PATH"]+"bin/examples/dynamic_obstacles/dynamic_obstacles_test","examples/test.yaml"]
                popen = subprocess.Popen(cmd)
                time.sleep(1)
    
    print("Done!")

