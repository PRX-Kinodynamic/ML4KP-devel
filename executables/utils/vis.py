import numpy as np
import os
from glob import glob
import pickle
from tqdm import tqdm
from matplotlib import pyplot as plt
from matplotlib.patches import Circle
import json

# data_path = "/media/dhruv/a7519aee-b272-44ae-a117-1f1ea1796db6/2024/NAMO/cylinder_data"
# data_path = "/media/dhruv/a7519aee-b272-44ae-a117-1f1ea1796db6/2024/NAMO/cylinder_data1"
data_path = "/media/dhruv/a7519aee-b272-44ae-a117-1f1ea1796db6/2024/NAMO/cylinder_data2"
data_path = "/media/dhruv/a7519aee-b272-44ae-a117-1f1ea1796db6/2024/NAMO/cylinder_env/level_2/0/failure"
folder = 'graphs/3'

do_xml_paths = True
plot_trajectories = False


xml_paths = []
success_count = 0
total_count = 0
# for k in tqdm(range(4)):
patches = []
for i in tqdm(sorted(os.listdir(data_path), key=lambda x: int(x))):
    total_count += 1
    path = os.path.join(data_path, i, 'metadata.json')
    with open(path, 'r') as f:
        data = json.load(f)
        xml_path = data['xml_path']
        xml_paths.append(xml_path)
        # goal = [float(x) for x in data['goal']]
        # success = int(data['success'])
        # num_steps = int(data['num_steps'])
        # success_count += success
    #         patches.append(Circle(goal, 0.1, color='g' if success else 'r'))

    # for patch in patches:
    #     plt.gca().add_patch(patch)

    # plt.xlim(-2, 2)
    # plt.ylim(-2, 2)
    # plt.axis('equal')
    # plt.axis('off')

    # if not os.path.exists(folder):
    #     os.makedirs(folder)
    # plt.savefig(f'{folder}/success_rate_{k}.png')
    # plt.close()

print(success_count, total_count, success_count/total_count)

if do_xml_paths:
    if os.path.exists('xml_paths.pkl'):
        with open('xml_paths.pkl', 'rb') as f:
            xml_paths_old = pickle.load(f)
    else:
        xml_paths_old = []

    xml_paths_final = list(set(xml_paths_old + xml_paths))
    print('-'*100)
    print(len(xml_paths_final))
    print('-'*100)
    with open('xml_paths.pkl', 'wb') as f:
        pickle.dump(xml_paths_final, f)

if plot_trajectories:
    fig = plt.figure()
    ax = fig.add_subplot(111)
    for k in range(33, 55):
        for i in sorted(os.listdir(data_path), key=lambda x: int(x))[289*k:289*(k+1)]:
            path = os.path.join(data_path, i, 'trajectory.txt')
            metadata_path = os.path.join(data_path, i, 'metadata.txt')
        with open(metadata_path, 'r') as f:
            print(f.read())
        lines = []
        with open(path, 'r') as f:
            for line in f:
                    lines.append([float(x) for x in line.strip().split(' ')[:-1]])
            lines = np.array(lines[:-1])
            ax.plot(lines[:, 2], lines[:, 3], color='r')

    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.set_aspect('equal')
    ax.axis('off')
    plt.show()
        
    
        