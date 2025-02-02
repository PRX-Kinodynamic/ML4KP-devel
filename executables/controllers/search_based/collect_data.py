import os
import random
from glob import glob
import argparse
from tqdm import tqdm

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser()
    parser.add_argument('--num_envs', type=int, default=100)
    args = parser.parse_args()
    model_dir = 'resources/models/mujoco_envs_empty_walls'
    model_paths = glob(os.path.join(model_dir, '*.xml'))
    
    for _ in tqdm(range(args.num_envs)):
        try:
            idx = random.randint(0, len(model_paths) - 1)
            model_path = model_paths[idx]
            # print(model_path)
            st = f'python executables/controllers/search_based/controller_planner.py --model_xml {model_path} --headless'
            os.system(st)
        except KeyboardInterrupt:
            print("Keyboard interrupt")
            break
