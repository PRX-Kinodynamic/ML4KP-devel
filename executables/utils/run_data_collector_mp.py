import os
import sys
import yaml
from tqdm import tqdm
import signal
import pickle
import multiprocessing
from functools import partial

def process_xml_file(xml_file, params_template, push_plan_yaml, command_line_args, log_dir):
    # Create a unique log file for each process
    log_file = os.path.join(log_dir, f'data_collection_log_{os.getpid()}.txt')
    os.system(f'touch {log_file}')
    
    # Get environment name from XML file (without .xml extension)
    env_config_name = os.path.splitext(xml_file)[0]
    
    # Create a copy of params for this process
    params = params_template.copy()
    params['xml_path'] = f"cylinder_env/level_1/{xml_file}"
    
    # Modify data_path to include environment name
    base_data_path = params['data_path']
    params['data_path'] = os.path.join(base_data_path, env_config_name)
    
    # Create unique push plan yaml for this process and environment
    process_yaml = f"{push_plan_yaml.split('.')[0]}_{env_config_name}.yaml"
    with open(process_yaml, 'w') as file:
        yaml.dump(params, file)

    yaml_command_line_args = f"{command_line_args}_{env_config_name}.yaml"
    
    try:
        # Pass the specific yaml file to interface
        return_code = os.system(f"./bin/examples/tamp/interface {yaml_command_line_args} > {log_file} 2>&1")
        if return_code != 0:
            print(f"Error processing {xml_file}, return code: {return_code}")
    finally:
        # Cleanup
        if os.path.exists(process_yaml):
            os.remove(process_yaml)

def main():
    push_plan_yaml = "resources/input_files/examples/tasks/push_plan.yaml"
    with open(push_plan_yaml, 'r') as file:
        params_template = yaml.safe_load(file)
    cmd_args = "examples/tasks/push_plan"

    xml_dir = "resources/models/cylinder_env/level_1"
    xml_files = [f for f in os.listdir(xml_dir) if f.endswith('.xml')][400:500]

    # Create a directory for log files
    log_dir = 'data_collection_logs'
    os.makedirs(log_dir, exist_ok=True)

    # Number of processes to use (adjust based on your CPU cores)
    
    num_processes = 6 # max(1, 4) # multiprocessing.cpu_count() - 1)  # Leave one core free

    try:
        # Create a pool of workers
        with multiprocessing.Pool(num_processes) as pool:
            # Create partial function with fixed arguments
            process_func = partial(
                process_xml_file,
                params_template=params_template,
                push_plan_yaml=push_plan_yaml,
                command_line_args=cmd_args,
                log_dir=log_dir
            )
            
            # Process files in parallel with progress bar
            list(tqdm(
                pool.imap_unordered(process_func, xml_files),
                total=len(xml_files),
                desc="Processing XML files"
            ))

    except KeyboardInterrupt:
        print("\nReceived keyboard interrupt. Cleaning up and exiting...")
        sys.exit(0)
    except Exception as e:
        print(f"An error occurred: {e}")
        sys.exit(1)
    finally:
        print("Done!")

if __name__ == '__main__':
    main()