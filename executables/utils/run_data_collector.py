import os
import sys
import yaml
from tqdm import tqdm
import signal
import pickle
push_plan_yaml = "resources/input_files/examples/tasks/push_plan.yaml"

with open(push_plan_yaml, 'r') as file:
    params = yaml.safe_load(file)

xml_dir = "resources/models/cylinder_env/random_start/level_1"
xml_files = [f for f in os.listdir(xml_dir) if f.endswith('.xml')]

log_file = 'data_collection_log.txt'
os.system(f'touch {log_file}')

with open('xml_paths.pkl', 'rb') as f:
    xml_paths = pickle.load(f)

try:
    for xml_file in tqdm(xml_files[:500]):
        params['xml_path'] = f"cylinder_env/random_start/level_1/{xml_file}"
        # if any([params['xml_path'] in path for path in xml_paths]):
        #     print(f"Skipping {xml_file} because it has already been collected")
        #     continue
        with open(push_plan_yaml, 'w') as file:
            yaml.dump(params, file)
        try:
            # Run the command and check return code
            return_code = os.system(f"./bin/examples/tamp/interface > {log_file} 2>&1")
            if return_code == 2:  # Check if process was interrupted
                print("\nReceived keyboard interrupt. Cleaning up and exiting...")
                break
        except KeyboardInterrupt:
            print("\nReceived keyboard interrupt. Cleaning up and exiting...")
            break

except KeyboardInterrupt:
    print("\nReceived keyboard interrupt. Cleaning up and exiting...")
    sys.exit(0)
except Exception as e:
    print(f"An error occurred: {e}")
    sys.exit(1)
finally:
    # Any cleanup code here if needed
    print("Done!")
