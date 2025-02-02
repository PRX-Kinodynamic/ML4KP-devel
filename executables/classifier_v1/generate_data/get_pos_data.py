from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
from glob import glob
from xml_object_parser import XMLParser
import numpy as np
import os
import yaml
import json
from tqdm import tqdm
folder = "/media/dhruv/a7519aee-b272-44ae-a117-1f1ea1796db6/2024/NAMO/bottleneck/dataset2/data"
DEST_FOLDER = "/media/dhruv/a7519aee-b272-44ae-a117-1f1ea1796db6/2024/NAMO/dataset_pos_1"


def get_model_config(name):
    model_path = os.path.join(models_folder, base_folder, f"{name}.xml")
    config_path = os.path.join(config_folder, base_folder, f"{name}.yaml")
    model_config = {
        'MJ_MODEL_PATH': model_path,
        'MJ_CONFIG_PATH': config_path,
        'MOVABLE_PREFIX': 'movable',
        'STATIC_PREFIX': ['static', 'wall']
    }
    return model_config

models_folder = "resources/models"
config_folder = "resources/input_files"
base_folder = "bottleneck"
name = "model1"

model_config = get_model_config(name)
xml_parser = XMLParser(model_config)
xml_parser.parse()

for movable in xml_parser.movable_bodies:
    print(movable.pos, movable.euler)

for static in xml_parser.static_bodies:
    print(static.pos, static.euler)