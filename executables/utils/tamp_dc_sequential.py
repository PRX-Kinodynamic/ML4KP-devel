import os
import sys
import yaml
from tqdm import tqdm
import random

random.seed(42)

def process_xml_file(xml_details, params_template, push_plan_yaml, command_line_args, log_dir):
    xml_dir, xml_file = xml_details
    env_config_name = os.path.splitext(xml_file)[0]
    
    # Run 30 times with different random seeds
    for seed in range(10):
        # Create a unique log file for each environment and seed
        log_file = os.path.join(log_dir, f'data_collection_log_{env_config_name}_seed{seed}.txt')
        os.system(f'touch {log_file}')
        
        # Create a copy of params for this environment
        params = params_template.copy()
        params['xml_path'] = f"{'/'.join(xml_dir.split('/')[2:])}/{xml_file}"
        print(params['xml_path'])
        
        # Set a unique random seed for each run
        params['random_seed'] = seed
        
        # Create unique push plan yaml for this environment and seed
        process_yaml = f"{push_plan_yaml.split('.')[0]}_{env_config_name}_seed{seed}.yaml"
        with open(process_yaml, 'w') as file:
            yaml.dump(params, file)

        yaml_command_line_args = f"{command_line_args}_{env_config_name}_seed{seed}.yaml"

        try:
            # Pass the specific yaml file to interface
            return_code = os.system(f"./bin/examples/namo/interface_namo {yaml_command_line_args} > {log_file} 2>&1")
            if return_code != 0:
                print(f"Error processing {xml_file} with seed {seed}, return code: {return_code}")
        finally:
            # Cleanup
            if os.path.exists(process_yaml):
                os.remove(process_yaml)

def main():
    push_plan_yaml = "resources/input_files/examples/tasks/tamp_plan.yaml"
    with open(push_plan_yaml, 'r') as file:
        params_template = yaml.safe_load(file)
    cmd_args = "examples/tasks/tamp_plan"

    xml_details = []
    
    # Configure which files to process
    a = 0
    b = 10
    dirs = [("resources/models/custom_walled_envs/empty", a, b)]

    for xml_dir, start_idx, end_idx in dirs:
        xml_details.extend([(xml_dir, f) for f in sorted(os.listdir(xml_dir), key=lambda x: int(x.split('.')[0].split('_')[-1])) if f.endswith('.xml')][start_idx:end_idx])

    # Create a directory for log files
    log_dir = 'data_collection_logs'
    os.makedirs(log_dir, exist_ok=True)

    print(f"Processing {len(xml_details)} XML files sequentially")
    
    # Process files sequentially with progress bar
    for xml_detail in tqdm(xml_details, desc="Processing XML files"):
        print(xml_detail)
        process_xml_file(
            xml_detail,
            params_template=params_template,
            push_plan_yaml=push_plan_yaml,
            command_line_args=cmd_args,
            log_dir=log_dir
        )

    print("Done!")

if __name__ == '__main__':
    main() 