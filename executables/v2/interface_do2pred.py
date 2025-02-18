### This interface pertains to the problem of predicting yes/no for a given scene configuration and the ability of the robot to reach the goal state
from problems.direct_objects import DirectObjects
from pathlib import Path
from methods.classifiers import GNNClassifier, MLPClassifier
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
    'simple_scene_simple_controller_direct_objects': {
        'class': DirectObjects,
        'config': 'executables/v2/problems/configs/direct_objects/simple_scene_simple_controller.yaml'
    }
}

method_map = {
    'gnn': {
        'config': 'executables/v2/methods/configs/gnn_config.yaml',
        'class': GNNClassifier
    },
    'mlp': {
        'config': 'executables/v2/methods/configs/mlp_config.yaml',
        'class': MLPClassifier
    }
}

def plot_predictions(predictions, file_name, fpr=None, classification_threshold=0.5):
    fig = plt.figure(figsize=(15, 5))
    ax1 = fig.add_subplot(131)
    ax2 = fig.add_subplot(132)
    ax3 = fig.add_subplot(133)
    success_pred = 0
    patch_output, patch_pred, patch_label = [], [], []
    object_patches = []
    for pred in predictions:
        folder = pred[0]
        output, pred, success = pred[1]
        # if int(folder.split('/')[-1]) % 2 != 0:
        #     continue
        metadata = {}
        with open(os.path.join(folder, "metadata.json"), "r") as f:
            metadata = json.load(f)
        goal = metadata["goal_state"][:2]
        goal_pt_patch = goal.copy()
        goal_pt_patch[0] -= 0.1
        goal_pt_patch[1] -= 0.1
        env = metadata["environment"]
        patch_output.append(Rectangle(goal_pt_patch, 0.2, 0.2, color='black', alpha=output))
        patch_pred.append(Rectangle(goal_pt_patch, 0.2, 0.2, color='green' if pred == 1.0 else 'red'))
        patch_label.append(Rectangle(goal_pt_patch, 0.2, 0.2, color= 'green' if float(success) == 1.0 else 'red'))
        for obj in env:
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
                
        success_pred += pred == float(success)
    success_rate = success_pred / len(predictions)
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
    fig.suptitle(f'Success Rate: {success_rate:.2f} FPR: {fpr:.2f}')
    
    plt.savefig(f'{file_name}.png')
    plt.close()

def run(problem: Problem, method: Method, load_model=False, verbose=False):
    classification_threshold = method.get_classification_threshold()
    # TODO: define this folder in a run config
    fpr_folder = f'fpr_predictions_{problem.get_problem_type()}/{problem.get_name()}_{method.get_name()}_{classification_threshold}'
    if not os.path.exists(fpr_folder):
        os.makedirs(fpr_folder)
    if load_model:  
        method.load_model()
    else:
        method.train(problem.get_training_data(), problem.get_evaluation_data(), problem.get_balance_ratio(), verbose)
        method.load_model()

    test_data = problem.get_test_data()
    
    print("Processing test data...")
    temp_dict = {}
    for data in test_data:
        metadata = {}
        with open(os.path.join(data, "metadata.json"), "r") as f:
            metadata = json.load(f)
        if metadata["xml_path"] not in temp_dict:
            temp_dict[metadata["xml_path"]] = []
        temp_dict[metadata["xml_path"]].append(data)


    pbar = tqdm(temp_dict.items(), total=len(temp_dict))
    
    true_positive_count = 0
    false_positive_count = 0
    false_negative_count = 0
    true_negative_count = 0

    local_fpr_list = []

    for xml_path, data in pbar:
        # this will change 
        local_fp_count = 0
        local_tn_count = 0
        
        predictions = method.predict_list(data) 
        for prediction in predictions:
            output, pred, success = prediction[1]
           
            # calculate metrics
            true_positive = pred == 1 and success == 1
            false_positive = pred == 1 and success == 0
            false_negative = pred == 0 and success == 1
            true_negative = pred == 0 and success == 0

            true_positive_count += true_positive
            false_positive_count += false_positive
            false_negative_count += false_negative
            true_negative_count += true_negative

            local_fp_count += false_positive
            local_tn_count += true_negative

        local_fpr = 0
        if local_fp_count + local_tn_count > 0:
            local_fpr = local_fp_count / (local_fp_count + local_tn_count)
        local_fpr_list.append((xml_path, local_fpr, predictions))
        pbar.update(1)
    
    pbar.close()

    accuracy = round((true_positive_count + true_negative_count) / (true_positive_count + true_negative_count + false_positive_count + false_negative_count), 2)

    false_positive_rate = round(false_positive_count / (false_positive_count + true_negative_count), 2)

    false_negative_rate = round(false_negative_count / (false_negative_count + true_positive_count), 2)

    precision = round(true_positive_count / (true_positive_count + false_positive_count), 2)

    recall = round(true_positive_count / (true_positive_count + false_negative_count), 2)
    
    f1_score = round(2 * true_positive_count / (2 * true_positive_count + false_positive_count + false_negative_count), 2)

    metrics = Metrics(accuracy=accuracy,
                      false_positive_rate=false_positive_rate,
                      false_negative_rate=false_negative_rate,
                      precision=precision,
                      recall=recall,
                      f1_score=f1_score)
    
    print(metrics)
    
    sorted_local_fpr_list = sorted(local_fpr_list, key=lambda x: x[1])[::-1]
    for i, (xml_path, fpr, predictions) in enumerate(sorted_local_fpr_list):
        # print(f"{i+1}. {xml_path}: {fpr:.2f}")
        file_name = Path(xml_path).stem
        plot_predictions(predictions, f'{fpr_folder}/{fpr:.2f}_{file_name}', fpr, classification_threshold)

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

            metrics = run(problem, method, load_model, verbose)
            all_metrics[f'{problem_name}_{method_name}'] = metrics

    # problem_config = convert_yaml_to_dict(problem_map[problem]['config'])
    # problem = problem_map[problem]['class'](problem_config)

    # method_config = convert_yaml_to_dict(method_map[method]['config'])
    # method = method_map[method]['class'](method_config)

    # metrics = run(problem, method, load_model)
    print(all_metrics)
    