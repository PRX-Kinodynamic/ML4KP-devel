from problems.direct_objects import MaskToImage
from pathlib import Path
from methods.classifiers import MLPMaskClassifier
from matplotlib import pyplot as plt
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
import matplotlib.patches as patches
from scipy.spatial.transform import Rotation as R
from matplotlib.patches import Rectangle

random.seed(42)
np.random.seed(42)

@dataclass
class Metrics:
    accuracy: float
    false_positive_rate: float
    false_negative_rate: float
    false_discovery_rate: float
    precision: float
    recall: float
    f1_score: float

problem_map = {
    'simple_scene_simple_controller': {
        'class': MaskToImage,
        'config': 'executables/v2/problems/configs/mask_to_imgs/simple_scene_simple_controller.yaml'
    }
}

method_map = {
    'mlp': {
        'config': 'executables/v2/methods/configs/mlp_mask_config.yaml',
        'class': MLPMaskClassifier
    },
}

def plot_predictions(predictions, file_name, env_props, goal_success, fpr=None, classification_threshold=0.5):
    fig = plt.figure(figsize=(15, 5))
    ax1 = fig.add_subplot(131)
    ax2 = fig.add_subplot(132)
    ax3 = fig.add_subplot(133)

    # Plot ground truth, thresholded prediction, and raw prediction
    patch_output, patch_pred, patch_label = [], [], []
    output_all, pred_all, target_all = predictions
    for idx in range(output_all.shape[0]):
        output, pred, goals = output_all[idx], pred_all[idx], goal_success[idx]

        goal = goals[:2]
        success = goals[2]
        goal_pt_patch = np.array(goal).copy()
        goal_pt_patch[0] -= 0.1
        goal_pt_patch[1] -= 0.1
        patch_output.append(Rectangle(goal_pt_patch, 0.2, 0.2, color='black', alpha=output))
        patch_pred.append(Rectangle(goal_pt_patch, 0.2, 0.2, color='green' if pred == 1.0 else 'red'))
        patch_label.append(Rectangle(goal_pt_patch, 0.2, 0.2, color= 'green' if float(success) == 1.0 else 'red'))

    for patch in patch_label:
        ax1.add_patch(patch)
    ax1.set_title('Ground Truth')

    for patch in patch_pred:
        ax2.add_patch(patch)
    ax2.set_title(f'Thresholded Prediction at {classification_threshold}')

    for patch in patch_output:
        ax3.add_patch(patch)
    ax3.set_title('Raw Prediction Values')
    
    # Add object outlines
    for obj in env_props:
        if 'obstacle' in obj['name'] or 'robot' in obj['name']:
            pos = obj['pos'][:2]
            quat = R.from_quat(obj['quat'], scalar_first=True).as_euler('xyz', degrees=True)
            size = obj['size'][:2]
            color = 'blue'
            if 'robot' in obj['name'] or 'movable' in obj['name']:
                color = 'yellow'
            
            # Add rectangle to each subplot
            for ax in [ax1, ax2, ax3]:
                rect_patch = patches.Rectangle(
                    (pos[0] - size[0], pos[1] - size[1]), 
                    size[0] * 2, size[1] * 2,
                    angle=quat[2],
                    rotation_point='center',
                    color=color,
                    alpha=0.5,
                    fill=False
                )
                ax.add_patch(rect_patch)

    
    # Set consistent axes limits
    for ax in [ax1, ax2, ax3]:
        ax.set_xlim(-2, 2)
        ax.set_ylim(-2, 2)

    # plt.colorbar()
    plt.savefig(f'{file_name}.png')
    plt.close()

def run(problem, method, load_model, verbose):
    prediction_folder = problem.get_problem_type()
    if not os.path.exists(prediction_folder):
        os.makedirs(prediction_folder)
    if load_model:  
        method.load_model()
    else:
        method.train(problem.get_training_data(), problem.get_evaluation_data(), None, verbose)
        method.load_model()

    test_data = problem.get_test_data()
    pbar = tqdm(test_data)

    # Metrics tracking
    tp, fp, fn, tn = 0, 0, 0, 0

    for datafile in pbar:
        output, pred, target = method.predict(datafile.as_posix())
        with open(datafile.as_posix(), 'r') as f:
            data = json.load(f)
        env_props = data['environment']
        xml_path = data['xml_path']
        goal_success = data['goal_success']

        # Convert to numpy for calculations
        pred_np = pred.cpu().numpy().flatten()
        target_np = target.cpu().numpy().flatten()

        # Calculate metrics
        tp += np.sum((pred_np > 0.5) & (target_np > 0.5))
        fp += np.sum((pred_np > 0.5) & (target_np <= 0.5))
        fn += np.sum((pred_np <= 0.5) & (target_np > 0.5))
        tn += np.sum((pred_np <= 0.5) & (target_np <= 0.5))

        predictions = (output.cpu().numpy(), pred.cpu().numpy(), target.cpu().numpy())
        file_name = os.path.join(prediction_folder, xml_path.split('/')[-1].replace('.xml', ''))
        
        plot_predictions(predictions, file_name, env_props, goal_success)
        pbar.update(1)

    # Calculate final metrics
    total = tp + tn + fp + fn
    accuracy = (tp + tn) / total if total > 0 else 0
    fpr = fp / (fp + tn) if (fp + tn) > 0 else 0
    fnr = fn / (fn + tp) if (fn + tp) > 0 else 0
    fdr = fp / (fp + tp) if (fp + tp) > 0 else 0
    precision = tp / (tp + fp) if (tp + fp) > 0 else 0
    recall = tp / (tp + fn) if (tp + fn) > 0 else 0
    f1_score = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0

    metrics = Metrics(accuracy, fpr, fnr, fdr, precision, recall, f1_score)
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

    prediction_folder = 'predictions'
    if not os.path.exists(prediction_folder):
        os.makedirs(prediction_folder)

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

            run(problem, method, load_model, verbose) 