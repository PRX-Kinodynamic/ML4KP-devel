import os
from pathlib import Path
from tqdm import tqdm
import mujoco
from scipy.spatial.transform import Rotation as R
import json
import numpy as np

from matplotlib import pyplot as plt

def save_data_from_objects(xml_path):
    model_props = {}

    spec = mujoco.MjSpec.from_file(xml_path)
    model = spec.compile()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    # for all geom extract the position and size
    for i in range(model.ngeom):
        geom = model.geom(i)
        geom_info = {}
        geom_info['size'] = geom.size.tolist()
        geom_info['rot'] = R.from_matrix(data.geom_xmat[i].reshape(3,3)).as_euler('xyz', degrees=False)[2]
        geom_info['pos'] = data.geom_xpos[i].tolist()
        model_props[geom.name] = geom_info
        
    return model_props

def get_success_fail_folders(data_path):
    
    success_folders = []
    fail_folders = []
    for folder in tqdm(os.listdir(data_path)):
        
        folder_path = os.path.join(data_path, folder)
        if os.path.isdir(folder_path):
            metadata_path = os.path.join(folder_path, "metadata.txt")
            with open(metadata_path, "r") as f:
                f.readline()# .split(":")[1].strip()
                f.readline()
                success = int(f.readline().split(':')[1].strip())
                # num_steps = f.readline() # .split(':')[1].strip())
            if success == 1:
                with open('eval_success_folders.txt', 'a') as new_file:
                    new_file.write(f"{folder}\n")
                # success_folders.append(folder)
            else:
                with open('eval_fail_folders.txt', 'a') as new_file:
                    new_file.write(f"{folder}\n")
                    # fail_folders.append(folder)
    return success_folders, fail_folders



def main(main_data_path, data_folders):
    for data_folder in data_folders:
        data_path = main_data_path / data_folder
        success_folders, fail_folders = get_success_fail_folders(data_path)
        break

if __name__ == "__main__":

    # to split the data into success and fail folders   
    # main_data_path = Path("/media/dhruv/a7519aee-b272-44ae-a117-1f1ea1796db6/2024/NAMO/cylinder_env/level_2")
    # data_folders = [ '0' ]

    # main(main_data_path, data_folders)

    # to extract the data from the mujoco using xml files
    resources_folder_temp = Path("resources/temp")
    resources_folder_temp.mkdir(parents=True, exist_ok=True)
    xml_folder = Path("resources/models/cylinder")
    for xml_file in tqdm(xml_folder.iterdir()):
        data = save_data_from_objects(xml_file.as_posix())
        with open(resources_folder_temp / f"{xml_file.stem}.json", 'w') as f:
            json.dump(data, f)
    # first_idx = 0
    # first_points = []
    # second_points = []
    # folders = os.listdir(os.path.join(main_data_path, data_folders[0]))
    # for folder in sorted(folders, key=lambda x: int(x)):
    #     metadata_path = os.path.join(main_data_path, data_folders[0], folder, "metadata.txt")
    #     with open(metadata_path, "r") as f:
    #         xml_path = Path(f.readline().split(":")[1].strip())
    #         if xml_path.stem == "env_config_173":
    #             print("folder", folder)
    #             trajectory_path = os.path.join(main_data_path, data_folders[0], folder, "trajectory.txt")
    #             with open(trajectory_path, "r") as f:
    #                 trajectory = f.readlines()

    #             first_points.append([float(x) for x in trajectory[1].split(' ')[:2]])
    #             try:
    #                 second_points.append([float(x) for x in trajectory[2].split(' ')[:2]])
    #             except:
    #                 print("no second point")
                # if first_idx == 0:
                #     first_idx += 1

    # first_points = np.array(first_points)
    # second_points = np.array(second_points)
    
    # plt.scatter(first_points[:, 0], first_points[:, 1], color='r')
    # plt.scatter(second_points[:, 0], second_points[:, 1], color='b')
    # plt.show()
    
                
            
            
        
        