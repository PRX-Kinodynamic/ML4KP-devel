from problems.direct_objects import DirectObjects
from pathlib import Path
from methods.classifiers import GNNClassifier, MLPClassifier
from matplotlib import pyplot as plt
from matplotlib.patches import Circle
from tqdm import tqdm
from dataclasses import dataclass
from problems.base_problem import Problem
from methods.base_method import Method
from utils import convert_yaml_to_dict
import os
import argparse

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
        'config': 'executables/v2/problems/configs/simple_scene_simple_controller.yaml'
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
    for pred in predictions:
        folder = pred[0]
        output, pred = pred[1]
        metadata = {}
        with open(folder + '/metadata.txt', 'r') as f:
            for line in f.readlines():
                key, value = line.strip().split(":")
                metadata[key.strip()] = value.strip()
        xml_path = Path(metadata["xml_path"])
        goal = [float(x) for x in metadata["goal_state"].strip().split()[:2]]
        patch_output.append(Circle(goal, 0.1, color='black', alpha=output))
        patch_pred.append(Circle(goal, 0.1, color='black', alpha=pred))
        patch_label.append(Circle(goal, 0.1, color='black', alpha= float(metadata['success'])))
        success_pred += pred == float(metadata['success'])
    success_rate = success_pred / len(predictions)
    for patch in patch_label:
        ax1.add_patch(patch)

    for patch in patch_pred:
        ax2.add_patch(patch)

    for patch in patch_output:
        ax3.add_patch(patch)

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
    fpr_folder = f'fpr_predictions/{problem.get_name()}_{method.get_name()}_{classification_threshold}'
    if not os.path.exists(fpr_folder):
        os.makedirs(fpr_folder)
    if load_model:  
        method.load_model()
    else:
        method.train(problem.get_training_data(), problem.get_evaluation_data(), problem.get_balance_ratio(), verbose)
        method.load_model()

    test_data = problem.get_test_data()
    pbar = tqdm(test_data, total=len(test_data))
    
    true_positive_count = 0
    false_positive_count = 0
    false_negative_count = 0
    true_negative_count = 0

    local_fpr_list = []

    for data in pbar:
        # this will change 
        local_fp_count = 0
        local_tn_count = 0

        predictions = method.predict_list(data) 
        for prediction in predictions:
            output, pred = prediction[1]
            metadata = {}
            with open(prediction[0] + '/metadata.txt', 'r') as f:
                for line in f.readlines():
                    key, value = line.strip().split(":")
                    metadata[key.strip()] = value.strip()

            success = int(metadata['success'])
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
        local_fpr_list.append((metadata['xml_path'], local_fpr, predictions))
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
        if fpr < 0.01:
            break

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
    