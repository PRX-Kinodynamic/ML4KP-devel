import argparse
import os
import sys
import yaml
import subprocess
from tqdm import tqdm
import mujoco

'''
python executables/utils/eval_namo.py --eval_type one_scene --object_strategy 1 --model custom_walled_envs/apr18_25/random_start_fixed_goal_one_env_1 --num_trials 50 --num_configs 30 --num_processes 1 
'''

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--eval_type", type=str, default="one_scene", choices=["one_scene", "many_scenes"])
    # parser.add_argument("--diffusion", type=bool, default=False)
    # parser.add_argument("--diffusion_goal", type=bool, default=False)
    parser.add_argument("--object_strategy", type=int, default=1, choices=[0, 1, 2, 3])
    parser.add_argument("--env_type", type=str, choices=["fixed", "random"])
    parser.add_argument("--model", type=str, default="custom_walled_envs/apr18_25/random_start_fixed_goal_one_env_1")
    parser.add_argument("--num_trials", type=int, default=50)
    parser.add_argument("--num_configs", type=int, default=1)
    parser.add_argument("--num_processes", type=int, default=1)
    args = parser.parse_args()
    return args

def load_yaml_file(file_path):
    with open(file_path, 'r') as f:
        return yaml.safe_load(f)

    
def process_xml(args_tuple):
    xml_path, yaml_params, base_folder, yaml_parent, yaml_folder, execution_cmd, num_trials = args_tuple
    # save temp yaml file
    model_file = xml_path.split('/')[-1].split('.')[0]
    results_folder = yaml_params["results_folder"] + "/" + model_file
    temp_yaml_file = os.path.join(yaml_folder, f"temp_{model_file}.yaml")
    temp_yaml_path = os.path.join(base_folder, yaml_parent, temp_yaml_file)
    
    # Create a copy of yaml_params to avoid modifying the shared dictionary
    yaml_params_copy = yaml_params.copy()
    yaml_params_copy["xml_path"] = xml_path
    
    
    model = mujoco.MjModel.from_xml_path(os.path.join(base_folder, "models", xml_path))
    data = mujoco.MjData(model)
    # check if model has a goal site within worldbody
    # Retrieve the site ID
    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'goal')
    # Access the position of the site
    if site_id != -1:
        robot_goal = model.site_pos[site_id][:2].tolist()
    del model, data
    
    yaml_params_copy["robot_goal"] = robot_goal
    yaml_params_copy["results_folder"] = results_folder
    
    with open(temp_yaml_path, 'w') as f:
        yaml.dump(yaml_params_copy, f)
    for i in tqdm(range(num_trials), desc=f"Running NAMO for {model_file}"):
        cmd = [execution_cmd, temp_yaml_file]
        subprocess.call(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def run_namo_eval(args):
    
    import multiprocessing as mp
    num_processes = 24
    
    execution_cmd = "./bin/examples/namo/interface_namo"
    base_folder = "resources"
    yaml_parent = "input_files"
    yaml_folder = "examples/tasks"
    yaml_file = "tamp_plan.yaml"
    yaml_path = os.path.join(base_folder, yaml_parent, yaml_folder, yaml_file)
    yaml_params = load_yaml_file(yaml_path)
    
    # set evaluate to true
    yaml_params["visualize"] = False
    yaml_params["evaluate"] = False
    # yaml_params["results_folder"] = "out/namo_results/" + args.eval_type + "/" + "dino_diffusion_many" + "_apr27"
    # if args.diffusion:
    #     yaml_params["diffusion"]["enabled"] = True
    # if args.diffusion_goal:
    #     yaml_params["diffusion"]["goal_enabled"] = True
    yaml_params["object_strategy"] = 1
    yaml_params["smoothing_enabled"] = False
    yaml_params["total_iter"] = 10

    # if args.eval_type == "one_scene":
    #     yaml_params["robot_goal"] = [2.5, 2.5]

    model_xml = []
    # print(os.path.join(base_folder, "models", args.model))
    if os.path.isdir(os.path.join(base_folder, "models", args.model)):
        for file in os.listdir(os.path.join(base_folder, "models", args.model)):
            if file.endswith(".xml"):
                model_xml.append(os.path.join(args.model, file))
    else:
        model_xml.append(args.model)
    
    num_configs = args.num_configs
    
    ## add multiprocessing here for this loop
    xml_files = sorted(model_xml, key=lambda x: int(x.split('/')[-1].split('.')[0].split('_')[-1]))[11:num_configs]
    
    args_list = [(xml_path, yaml_params, base_folder, yaml_parent, yaml_folder, execution_cmd, args.num_trials) for xml_path in xml_files]
    
    with mp.Pool(args.num_processes) as pool:
        pool.map(process_xml, args_list)
    

if __name__ == "__main__":
    args = parse_args() 
    run_namo_eval(args)