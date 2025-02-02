from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
from glob import glob
from xml_object_parser import XMLParser
import numpy as np
import os
import yaml
import json
from tqdm import tqdm
import pandas as pd
import imageio 

folder = "/media/dhruv/a7519aee-b272-44ae-a117-1f1ea1796db6/2024/NAMO/bottleneck/dataset3/data"
DEST_FOLDER = "/media/dhruv/a7519aee-b272-44ae-a117-1f1ea1796db6/2024/NAMO/dataset_3"

data_records = []
images = []
def get_model_config(name):
    model_path = os.path.join(models_folder, base_folder, f"{name}.xml")
    config_path = os.path.join(config_folder, base_folder, f"{name}.yaml")
    model_config = {
    'MJ_MODEL_PATH': model_path,
    'MOVABLE_PREFIX': 'movable',
        'STATIC_PREFIX': ['static', 'wall']
    }
    return model_config

models_folder = "resources/models"
config_folder = "resources/input_files"
base_folder = "bottleneck"
name = "model3"

model_config = get_model_config(name)
xml_parser = XMLParser(model_config)
xml_parser.parse()
# TODO: one fixed size for all plots (be it any model)

fig, ax = plt.subplots(figsize=((xml_parser.x_max - xml_parser.x_min)*5, (xml_parser.y_max - xml_parser.y_min)*5))
for trial_dir in tqdm(glob(os.path.join(folder, '*'))):
    try:
        ax.clear()
        for body in xml_parser.movable_bodies:
            body.plot(ax, movable=True)

        for body in xml_parser.static_bodies:
            body.plot(ax, movable=False)

        ax.set_xlim([xml_parser.x_min, xml_parser.x_max])
        ax.set_ylim([xml_parser.y_min, xml_parser.y_max])

        info_file = os.path.join(trial_dir, f'{trial_dir.split("/")[-1]}_info.json')
        with open(info_file, 'r') as f:
            info = json.load(f)

        # plot robot position as rectangle patch
        ax.add_patch(Rectangle((info['robot_position'][0]-0.05, info['robot_position'][1]-0.05), 0.1, 0.1, color='blue'))
        # plot goal as rectangle patch
        ax.add_patch(Rectangle((info['goal'][0]-0.05, info['goal'][1]-0.05), 0.1, 0.1, color='blue'))

        if info['success']:
            # plot line from robot to goal
            ax.plot([info['robot_position'][0], info['goal'][0]], [info['robot_position'][1], info['goal'][1]], color='green')
        else:
            ax.plot([info['robot_position'][0], info['goal'][0]], [info['robot_position'][1], info['goal'][1]], color='red')
        
        # no ticks
        ax.set_xticks([])
        ax.set_yticks([])
        ax.axis('off')

        if info['success']:
            dest_path = os.path.join(DEST_FOLDER, 'positive')
        else:
            dest_path = os.path.join(DEST_FOLDER, 'negative')
        
        if not os.path.exists(dest_path):
            os.makedirs(dest_path)
        # count existing images in dest_path and add 1 to it
        existing_images = len(glob(os.path.join(dest_path, '*.png')))
        dest_path = os.path.join(dest_path, f"{existing_images+1}.png")
        
        fig.canvas.draw()
        image = np.frombuffer(fig.canvas.tostring_rgb(), dtype=np.uint8)
        image = image.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        images.append(image)

        if len(images) > 50:
            break


        data_records.append({
            'image_path': dest_path,
            'robot_x': info['robot_position'][0],
            'robot_y': info['robot_position'][1],
            'goal_x': info['goal'][0],
            'goal_y': info['goal'][1],
            'success': info['success']
        })

        # plt.savefig(dest_path, dpi=300, bbox_inches='tight', pad_inches=0)
        # plt.pause(1.0)
    except Exception as e:
        print(e)
        # os.remove(os.path.join(trial_dir))


gif_path = os.path.join(DEST_FOLDER, 'visualization.gif')
imageio.mimsave(gif_path, images, fps=2)  # Adjust fps as needed

plt.close()

# Save the collected data to CSV
# df = pd.DataFrame(data_records)
# csv_path = os.path.join(DEST_FOLDER, 'positions.csv')
# df.to_csv(csv_path, index=False)


