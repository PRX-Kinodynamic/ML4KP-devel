from problems.direct_objects import DirectObjectsToRadius
from methods.regressors import GNNRadiusRegressor, MLPRadiusRegressor
from matplotlib import pyplot as plt
from tqdm import tqdm
from dataclasses import dataclass
from utils import convert_yaml_to_dict
import os
import argparse
import numpy as np
import json
from matplotlib.patches import Circle, Rectangle
from scipy.spatial.transform import Rotation as R
from utils import get_rectangle_corners

@dataclass
class Metrics:
    mse: float
    mae: float
    max_error: float
    r2_score: float
    avg_false_discovery_rate: float  # Average FDR across all test points (FP / points in circle)
    max_false_discovery_rate: float  # Worst case FDR
    avg_false_positive_rate: float   # Average FPR across all test points (FP / negative cases)
    max_false_positive_rate: float   # Worst case FPR
    median_false_discovery_rate: float  # Median FDR
    median_false_positive_rate: float  # Median FPR

problem_map = {
    'simple_scene_radius': {
        'class': DirectObjectsToRadius,
        'config': 'executables/v2/problems/configs/direct_objects_to_radius/simple_scene_simple_controller.yaml'
    }
}

method_map = {
    'gnn': {
        'config': 'executables/v2/methods/configs/gnn_radius_config.yaml',
        'class': GNNRadiusRegressor
    },
    'mlp': {
        'config': 'executables/v2/methods/configs/mlp_radius_config.yaml',
        'class': MLPRadiusRegressor
    }
}

def is_point_in_circle(point, center, radius):
    """Check if a point lies within or on a circle"""
    return np.linalg.norm(point - center) <= radius

def floor_to_step(value, step=0.05):
    """Floor the value to the nearest step"""
    return np.floor(value / step) * step

def evaluate_prediction(pred_radius, data):
    """
    Evaluate prediction metrics for a single prediction
    Returns: (false_discovery_rate, false_positive_rate, num_false_positives, total_in_circle)
    
    FDR = FP / (points in predicted circle) = FP / (FP + TP)
    FPR = FP / (all negative cases) = FP / (FP + TN)
    """
    robot_pos = None
    for obj in data['environment']:
        if 'robot' in obj['name']:
            robot_pos = np.array(obj['pos'][:2])
            break
    
    goal_points = np.array(data['goal_success'])[:, :2]
    success_flags = np.array(data['goal_success'])[:, 2]

    # pred_radius = floor_to_step(pred_radius)
    
    points_in_circle = np.array([
        is_point_in_circle(point, robot_pos, pred_radius) 
        for point in goal_points
    ])
    
    # Calculate False Positives
    false_positives = np.sum(points_in_circle & ~success_flags.astype(bool))
    
    # Calculate total points in circle (for FDR)
    total_in_circle = np.sum(points_in_circle)
    
    # Calculate total negative cases (for FPR)
    total_negative_cases = np.sum(~success_flags.astype(bool))
    
    # Calculate both rates
    fdr = false_positives / total_in_circle if total_in_circle > 0 else 0
    fpr = false_positives / total_negative_cases if total_negative_cases > 0 else 0
    
    return fdr, fpr, false_positives, total_in_circle

def plot_predictions(predictions, file_name):
    true_values = [p[1] for p in predictions]
    predicted_values = [p[0] for p in predictions]
    fp_rates = [p[2] for p in predictions]  # Add FP rates to plot
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
    
    # Scatter plot of predictions
    ax1.scatter(true_values, predicted_values, alpha=0.5, c=fp_rates, cmap='viridis')
    ax1.plot([min(true_values), max(true_values)], [min(true_values), max(true_values)], 'r--')
    ax1.set_xlabel('True Radius')
    ax1.set_ylabel('Predicted Radius')
    ax1.set_title('Predicted vs True Radius (color = FP rate)')
    plt.colorbar(ax1.collections[0], ax=ax1, label='False Positive Rate')
    
    # Histogram of false positive rates
    ax2.hist(fp_rates, bins=50)
    ax2.set_xlabel('False Positive Rate')
    ax2.set_ylabel('Count')
    ax2.set_title('Distribution of False Positive Rates')
    
    plt.tight_layout()
    plt.savefig(f'{file_name}.png')
    plt.close()

def plot_high_fpr_cases(predictions, data_files, prediction_folder, fpr_threshold=0.005):
    """Plot cases where false positive rate is higher than threshold"""
    high_fpr_cases = []
    
    # Create goal_lists folder if it doesn't exist
    goal_lists_folder = os.path.join(prediction_folder, 'goal_lists')
    if not os.path.exists(goal_lists_folder):
        os.makedirs(goal_lists_folder)
    
    # Collect high FPR cases
    for (pred_radius, true_radius, fp_rate), data_file in zip(predictions, data_files):
        if fp_rate > fpr_threshold:
            with open(data_file, 'r') as f:
                data = json.load(f)
            # Floor the predicted radius for visualization
            # pred_radius = floor_to_step(pred_radius)
            xml_path = data['xml_path']
            xml_name = os.path.basename(xml_path).replace('.xml', '')
            high_fpr_cases.append({
                'pred_radius': pred_radius,
                'true_radius': true_radius,
                'fp_rate': fp_rate,
                'data': data,
                'xml_name': xml_name
            })
    
    print(f"Found {len(high_fpr_cases)} cases with FPR > {fpr_threshold}")
    
    # Sort by FPR
    high_fpr_cases = sorted(high_fpr_cases, key=lambda x: x['fp_rate'], reverse=True)
    
    # Plot each case
    for i, case in enumerate(high_fpr_cases):
        fig, ax = plt.subplots(figsize=(10, 10))
        
        # Get robot position
        robot_pos = None
        for obj in case['data']['environment']:
            pos = np.array(obj['pos'][:2])
            rot = R.from_quat(obj['quat'], scalar_first=True).as_euler('xyz')[2]
            size = np.array(obj['size'][:2]) * 2  # Full size
            
            # Create rectangle patch
            rect = Rectangle(
                pos - size/2,  # bottom-left corner
                size[0], size[1],  # width, height
                angle=np.degrees(rot),
                rotation_point='center'
            )
            
            if 'robot' in obj['name']:
                robot_pos = pos
                rect.set_facecolor('lightblue')
                rect.set_edgecolor('blue')
                rect.set_alpha(0.5)
                rect.set_label('Robot')
            elif 'movable' in obj['name']:
                rect.set_facecolor('yellow')
                rect.set_edgecolor('orange')
                rect.set_alpha(0.3)
                rect.set_label('Movable Obstacle')
            elif 'static' in obj['name']:
                rect.set_facecolor('red')
                rect.set_edgecolor('darkred')
                rect.set_alpha(0.3)
                rect.set_label('Static Obstacle')
            
            ax.add_patch(rect)
        
        # Plot goal points and collect unsuccessful goals
        goal_points = np.array(case['data']['goal_success'])
        successes = goal_points[:, 2].astype(bool)
        
        # Plot successful points in green
        success_points = goal_points[successes]
        ax.scatter(success_points[:, 0], success_points[:, 1], 
                  c='green', alpha=0.5, label='Success', s=20)
        
        # Plot failure points in red
        failure_points = goal_points[~successes]
        ax.scatter(failure_points[:, 0], failure_points[:, 1], 
                  c='red', alpha=0.5, label='Failure', s=20)
        
        # Save unsuccessful goals to file
        unsuccessful_goals = goal_points[~successes][:, :2]
        goal_list_path = os.path.join(goal_lists_folder, f"{case['xml_name']}.txt")
        np.savetxt(goal_list_path, unsuccessful_goals, fmt='%.6f', delimiter=',')
        
        # Plot predicted circle (solid blue)
        pred_circle = Circle(robot_pos, case['pred_radius'], 
                           fill=False, linestyle='-', color='blue', 
                           label=f'Predicted (r={case["pred_radius"]:.2f})')
        ax.add_patch(pred_circle)
        
        # Plot true circle (dotted green)
        true_circle = Circle(robot_pos, case['true_radius'], 
                           fill=False, linestyle=':', color='green', 
                           label=f'True (r={case["true_radius"]:.2f})')
        ax.add_patch(true_circle)
        
        # Set plot properties
        ax.set_aspect('equal')
        ax.grid(True)
        
        # Set axis limits with some padding
        all_points = np.vstack([goal_points[:, :2], robot_pos])
        min_xy = np.min(all_points, axis=0) - 0.5
        max_xy = np.max(all_points, axis=0) + 0.5
        ax.set_xlim(min_xy[0], max_xy[0])
        ax.set_ylim(min_xy[1], max_xy[1])
        
        # Add legend and title with XML filename
        ax.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        ax.set_title(f'Scene: {case["xml_name"]}\nFPR: {case["fp_rate"]:.3f}\n'
                    f'Pred: {case["pred_radius"]:.2f}, True: {case["true_radius"]:.2f}')
        
        # Adjust layout to prevent legend cutoff
        plt.tight_layout()
        
        # Save plot
        plt.savefig(os.path.join(prediction_folder, f'high_fpr_case_{case["xml_name"]}.png'), 
                   bbox_inches='tight', dpi=150)
        plt.close()

def run(problem, method, load_model=False, verbose=False):
    prediction_folder = f'predictions/radius1/{problem.get_name()}_{method.get_name()}'
    if not os.path.exists(prediction_folder):
        os.makedirs(prediction_folder)

    if load_model:
        method.load_model()
    else:
        method.train(problem.get_training_data(), problem.get_evaluation_data(), verbose=verbose)
        method.load_model()

    test_data = problem.get_test_data()
    predictions = []
    false_discovery_rates = []
    false_positive_rates = []
    data_files = []
    
    pbar = tqdm(test_data)
    for data_file in pbar:
        with open(data_file, 'r') as f:
            data = json.load(f)
            
        pred_radius, true_radius = method.predict(data_file)
        fdr, fpr, num_fp, total_in_circle = evaluate_prediction(pred_radius, data)
        
        predictions.append((pred_radius, true_radius, fdr))  # Keep FDR for existing visualizations
        false_discovery_rates.append(fdr)
        false_positive_rates.append(fpr)
        data_files.append(data_file)
        
        if verbose:
            pbar.set_description(f"FDR: {fdr:.3f}, FPR: {fpr:.3f} ({num_fp}/{total_in_circle})")
        pbar.update(1)
    
    # Calculate metrics
    true_values = np.array([p[1] for p in predictions])
    predicted_values = np.array([p[0] for p in predictions])
    
    mse = np.mean((true_values - predicted_values) ** 2)
    mae = np.mean(np.abs(true_values - predicted_values))
    max_error = np.max(np.abs(true_values - predicted_values))
    
    # R² score
    ss_res = np.sum((true_values - predicted_values) ** 2)
    ss_tot = np.sum((true_values - np.mean(true_values)) ** 2)
    r2 = 1 - (ss_res / ss_tot)
    
    # Update metrics with both rates
    metrics = Metrics(
        mse=mse, 
        mae=mae, 
        max_error=max_error, 
        r2_score=r2,
        avg_false_discovery_rate=np.mean(false_discovery_rates),
        max_false_discovery_rate=np.max(false_discovery_rates),
        median_false_discovery_rate=np.median(false_discovery_rates),
        avg_false_positive_rate=np.mean(false_positive_rates),
        max_false_positive_rate=np.max(false_positive_rates),
        median_false_positive_rate=np.median(false_positive_rates)
    )
    print(metrics)
    
    plot_predictions(predictions, f'{prediction_folder}/predictions')
    plot_high_fpr_cases(predictions, data_files, prediction_folder)
    
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

            run(problem, method, args.load_model, args.verbose) 