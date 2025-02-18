### This interface pertains to the problem of predicting a 2D reachability mask from a given scene configuration and the robot's initial state.

from problems.direct_objects import DirectObjectsToImage
from pathlib import Path
from methods.classifiers import GNNSpatialClassifier
from matplotlib import pyplot as plt
from matplotlib.patches import Circle, Rectangle
from tqdm import tqdm
from dataclasses import dataclass
from problems.base_problem import Problem
from methods.base_method import Method
from utils import convert_yaml_to_dict
import os
import argparse
import json
import random
import numpy as np
from scipy.spatial.transform import Rotation as R
random.seed(42)
np.random.seed(42)

@dataclass
class Metrics:
    accuracy: float
    false_positive_rate: float
    false_negative_rate: float
    precision: float
    recall: float
    f1_score: float

problem_map = {
    'simple_scene_simple_controller': {
        'class': DirectObjectsToImage,
        'config': 'executables/v2/problems/configs/direct_objects_to_imgs/simple_scene_simple_controller.yaml'
    }
}

method_map = {
    'gnn': {
        'config': 'executables/v2/methods/configs/gnn_spatial_config.yaml',
        'class': GNNSpatialClassifier
    },
}

def plot_predictions(predictions, file_name, env_props, fpr=None, classification_threshold=0.5):
    fig = plt.figure(figsize=(15, 5))
    ax1 = fig.add_subplot(131)
    ax2 = fig.add_subplot(132)
    ax3 = fig.add_subplot(133)
    success_pred = 0
    patch_output, patch_pred, patch_label = [], [], []
    object_patches = []

    output_all, pred_all, target_all = predictions
    for idx in range(output_all.shape[0]):
        output, pred, goal_success = output_all[idx], pred_all[idx], target_all[idx]

        goal = goal_success[:2]
        success = goal_success[2]

        goal_pt_patch = goal.copy()
        goal_pt_patch[0] -= 0.1
        goal_pt_patch[1] -= 0.1
        patch_output.append(Rectangle(goal_pt_patch, 0.2, 0.2, color='black', alpha=output))
        patch_pred.append(Rectangle(goal_pt_patch, 0.2, 0.2, color='green' if pred == 1.0 else 'red'))
        patch_label.append(Rectangle(goal_pt_patch, 0.2, 0.2, color= 'green' if float(success) == 1.0 else 'red'))
    #     success_pred += pred == float(success)

    # success_rate = success_pred / len(predictions)

    for obj in env_props:
        if 'obstacle' in obj['name'] or 'robot' in obj['name']:
            pos = obj['pos'][:2]
            quat = R.from_quat(obj['quat'], scalar_first=True).as_euler('xyz', degrees=True)
            size = obj['size'][:2]
            color = 'blue'
            if 'robot' in obj['name'] or 'movable' in obj['name']:
                color = 'yellow'
            
            # 3 patches for 3 plots, matplotlib does not have copy on patches
            obj_patch = []
            for _ in range(3):
                rect_patch = Rectangle((pos[0] - size[0], pos[1] - size[1]), size[0] * 2, size[1] * 2, angle=quat[2], rotation_point='center', color=color, alpha=0.5, fill=False)
                obj_patch.append(rect_patch)
            object_patches.append(obj_patch)
    
    for patch in patch_label:
        ax1.add_patch(patch)

    for patch in patch_pred:
        ax2.add_patch(patch)

    for patch in patch_output:
        ax3.add_patch(patch)

    for patch in object_patches:
        ax1.add_patch(patch[0])
        ax2.add_patch(patch[1])
        ax3.add_patch(patch[2])

    ax1.set_xlim(-2, 2)
    ax1.set_ylim(-2, 2)
    ax1.set_title('Ground Truth')
    ax2.set_xlim(-2, 2)
    ax2.set_ylim(-2, 2)
    ax2.set_title(f'Prediction > {classification_threshold}')
    ax3.set_xlim(-2, 2)
    ax3.set_ylim(-2, 2)
    ax3.set_title('Prediction')
    # fig.suptitle(f'Success Rate: {success_rate:.2f} FPR: {fpr:.2f}')
    
    plt.savefig(f'{file_name}.png')
    plt.close()


def run(problem, method, load_model, verbose):
    prediction_folder = "predictions/dotoimg"
    if not os.path.exists(prediction_folder):
        os.makedirs(prediction_folder)
    if load_model:  
        method.load_model()
    else:
        method.train(problem.get_training_data(), problem.get_evaluation_data(), problem.get_balance_ratio(), verbose)
        method.load_model()

    test_data = problem.get_test_data()

    tp, fp, fn, tn = 0, 0, 0, 0

    pbar = tqdm(test_data)

    for datafile in pbar:
        output, pred, target = method.predict(datafile.as_posix())
        with open(datafile.as_posix(), 'r') as f:
            data = json.load(f)
        env_props = data['environment']
        goal_success = np.array(data['goal_success'])
        xml_path = data['xml_path']

        # Convert to numpy arrays and flatten
        pred = pred.cpu().numpy().flatten()
        goal_success = goal_success[:, 2].flatten()  # Get only success values

        # Calculate metrics
        tp += np.sum((pred == 1) & (goal_success == 1))
        fp += np.sum((pred == 1) & (goal_success == 0))
        fn += np.sum((pred == 0) & (goal_success == 1))
        tn += np.sum((pred == 0) & (goal_success == 0))

        predictions = (output.cpu().numpy().flatten(), pred, goal_success)

        file_name = os.path.join(prediction_folder, xml_path.split('/')[-1].replace('.xml', ''))
        # if np.random.uniform(0, 1) < 0.3:
        #     plot_predictions(predictions, file_name, env_props)
        pbar.update(1)
        
    # Calculate final metrics
    total = tp + tn + fp + fn
    accuracy = (tp + tn) / total if total > 0 else 0
    fpr = fp / (fp + tn) if (fp + tn) > 0 else 0
    fnr = fn / (fn + tp) if (fn + tp) > 0 else 0
    recall = tp / (tp + fn) if (tp + fn) > 0 else 0
    precision = tp / (tp + fp) if (tp + fp) > 0 else 0
    f1_score = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0

    metrics = Metrics(accuracy, fpr, fnr, precision, recall, f1_score)
    print(metrics)  
    return metrics

def arg_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('--load_model', action='store_true')
    parser.add_argument('--problem', type=str)
    parser.add_argument('--method', type=str)
    parser.add_argument('--verbose', action='store_true')
    return parser.parse_args()

if __name__ == '__main__':
    args = arg_parser()
    load_model = args.load_model
    verbose = args.verbose
    # problem = args.problem
    # method = args.method

    prediction_folder = 'predictions'
    if not os.path.exists(prediction_folder):
        os.makedirs(prediction_folder)

    all_metrics = {}

    for problem_name in problem_map:
        if args.problem is not None and problem_name != args.problem:
            continue
        for method_name in method_map:
            if args.method is not None and method_name != args.method:
                continue
            print(f'Running {problem_name} with {method_name}')
            problem_config = convert_yaml_to_dict(problem_map[problem_name]['config'])
            problem = problem_map[problem_name]['class'](problem_name, problem_config)

            method_config = convert_yaml_to_dict(method_map[method_name]['config'])
            method = method_map[method_name]['class'](method_name, method_config)

            # metrics = 
            run(problem, method, load_model, verbose)
            # all_metrics[f'{problem_name}_{method_name}'] = metrics
    
    # print(all_metrics)
