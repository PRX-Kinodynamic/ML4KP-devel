import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import torch
import random
from pathlib import Path

from problems.img_to_img import ImgToImg
from methods.pixel_wise_binary_classification import PixelWiseBinaryClassification
from utils import convert_yaml_to_dict  # Import the utility function

# Problem configuration mapping
problem_map = {
    'single_env_goal': {
        'class': ImgToImg,
        'config': 'executables/v3/problems/configs/single_env_goal.yaml'
    }
}

# Method configuration mapping
method_map = {
    'pixel_wise_binary': {
        'class': PixelWiseBinaryClassification,
        'config': 'executables/v3/methods/configs/pixel_wise_binary_clf.yaml'
    }
}

def visualize_prediction(input_img, target_mask, pred_probs, save_path=None):
    """
    Visualize the input image, target mask, and predictions (both raw probabilities and thresholded).
    
    Args:
        input_img: Input image
        target_mask: Target mask
        pred_probs: Predicted probabilities (unthresholded)
        save_path: Path to save the visualization
    """
    plt.figure(figsize=(12, 8))
    
    # Plot input image
    plt.subplot(2, 2, 1)
    plt.imshow(input_img)
    plt.title('Input Image')
    plt.axis('off')
    
    # Plot target mask
    plt.subplot(2, 2, 2)
    plt.imshow(target_mask)
    plt.title('Ground Truth Mask')
    plt.axis('off')
    
    # Plot predicted probabilities (unthresholded)
    plt.subplot(2, 2, 3)
    # input_img[:, :, 2] = pred_probs[:, :, 0]
    plt.imshow(pred_probs[:, :, 0])
    plt.title('Prediction (Raw Probabilities)')
    plt.colorbar(fraction=0.046, pad=0.04)
    plt.axis('off')
    
    # Plot thresholded prediction
    plt.subplot(2, 2, 4)
    plt.imshow((pred_probs > 0.5).astype(float))
    plt.title('Prediction (Thresholded)')
    plt.axis('off')
    
    plt.tight_layout()
    
    if save_path:
        plt.savefig(save_path, dpi=150)
        plt.close()
    else:
        plt.show()

def run_quick_preview(problem, method, num_samples=10, load_model=True, verbose=True):
    """
    Run a quick visualization preview on random test samples.
    
    Args:
        problem: Problem instance
        method: Method instance
        num_samples: Number of random samples to visualize
        load_model: Whether to load a pre-trained model
        verbose: Whether to print progress
    """
    # Create prediction folder
    preview_folder = f"predictions/preview/{problem.get_name()}_{method.get_name()}"
    os.makedirs(preview_folder, exist_ok=True)
    
    
    # Get test data
    test_data = problem.get_test_data()
    
    if len(test_data) == 0:
        print("No test data found!")
        return
    
    # Select random samples
    if num_samples > len(test_data):
        num_samples = len(test_data)
        if verbose:
            print(f"Only {num_samples} test samples available.")
    
    # Choose random indices
    random_indices = random.sample(range(len(test_data)), num_samples)
    
    if verbose:
        print(f"Generating visualizations for {num_samples} random test samples...")
    
    # Process selected samples
    for i, idx in enumerate(random_indices):
        print(test_data[idx])
        
        # Get model prediction with raw probabilities
        input_img, probs, pred, target_tensor = method.predict(test_data[idx])
        print(input_img.shape, probs.squeeze(0).shape, pred.shape, target_tensor.shape)
        probs = torch.softmax(probs, dim=2).cpu().squeeze(0).permute(1, 2, 0).numpy()
        pred = pred.cpu().squeeze(0).permute(1, 2, 0).numpy()
        target_tensor = target_tensor.cpu().squeeze(0).permute(1, 2, 0).numpy()
        print(input_img.shape, probs.shape, pred.shape, target_tensor.shape)
        
        target_tensor_new = np.zeros((target_tensor.shape[0], target_tensor.shape[1], 3))
        target_tensor_new[:, :, :1] = target_tensor
        
        probs_new = np.zeros((probs.shape[0], probs.shape[1], 3))
        probs_new[:, :, :1] = probs
        
        pred_new = np.zeros((pred.shape[0], pred.shape[1], 3))
        pred_new[:, :, :1] = pred
        
        # For model outputs, check if we got a tuple of (probs, pred, target)
        # if isinstance(model_output, tuple) and len(model_output) >= 2:
        #     pred_probs = model_output[0].cpu().numpy().squeeze()
        # else:
        #     # If the predict method already returns the binary prediction
        #     pred_probs = model_output
        
        
        
        # Visualize and save
        save_path = os.path.join(preview_folder, f"sample_{i}.png")
        visualize_prediction(input_img, target_tensor_new, probs_new, save_path)
        
        if verbose:
            print(f"Saved visualization {i+1}/{num_samples} to {save_path}")
    
    print(f"\nAll visualizations saved to {preview_folder}")

def run_full_evaluation(problem, method, load_model, verbose):
    """
    Run a complete evaluation on all test data and calculate metrics.
    
    Args:
        problem: Problem instance
        method: Method instance
        load_model: Whether to load a pre-trained model
        verbose: Whether to print detailed progress
    """
    # Create prediction folder
    prediction_folder = f"predictions/full_eval/{problem.get_name()}_{method.get_name()}"
    os.makedirs(prediction_folder, exist_ok=True)
    
    # Load or train model
    if load_model:  
        method.load_model()
    else:
        if verbose:
            print("Training model...")
        method.train(problem.get_training_data(), problem.get_evaluation_data(), 
                     problem.get_balance_ratio(), verbose)
    
    # Run predictions on test data
    if verbose:
        print("Processing test data...")
    
    test_data = problem.get_test_data()
    all_predictions = []
    all_targets = []
    
    pbar = tqdm(test_data) if verbose else test_data
    
    for i, (input_img, target_mask) in enumerate(pbar):
        # Predict mask
        pred_mask = method.predict(input_img)
        
        # If we get a tuple back with raw probabilities, use the thresholded version
        if isinstance(pred_mask, tuple) and len(pred_mask) >= 2:
            pred_probs = pred_mask[0].cpu().numpy().squeeze()
            pred_mask = (pred_probs > 0.5).astype(float)
        
        # Save for metrics calculation
        all_predictions.append(pred_mask)
        all_targets.append(target_mask)
    
    # Calculate metrics
    tp = fp = fn = tn = 0
    for pred, target in zip(all_predictions, all_targets):
        pred_flat = pred.flatten() > 0.5
        target_flat = target.flatten() > 0.5
        
        tp += np.sum((pred_flat) & (target_flat))
        fp += np.sum((pred_flat) & ~(target_flat))
        fn += np.sum(~(pred_flat) & (target_flat))
        tn += np.sum(~(pred_flat) & ~(target_flat))
    
    # Calculate derived metrics
    total = tp + tn + fp + fn
    accuracy = (tp + tn) / total if total > 0 else 0
    precision = tp / (tp + fp) if (tp + fp) > 0 else 0
    recall = tp / (tp + fn) if (tp + fn) > 0 else 0
    f1_score = 2 * (precision * recall) / (precision + recall) if (precision + recall) > 0 else 0
    iou = tp / (tp + fp + fn) if (tp + fp + fn) > 0 else 0
    
    # Display metrics
    print("\nEvaluation Metrics:")
    print(f"Accuracy:  {accuracy:.4f}")
    print(f"Precision: {precision:.4f}")
    print(f"Recall:    {recall:.4f}")
    print(f"F1 Score:  {f1_score:.4f}")
    print(f"IoU:       {iou:.4f}")
    
    # Save metrics to file
    metrics_path = os.path.join(prediction_folder, "metrics.txt")
    with open(metrics_path, 'w') as f:
        f.write(f"Accuracy:  {accuracy:.4f}\n")
        f.write(f"Precision: {precision:.4f}\n")
        f.write(f"Recall:    {recall:.4f}\n")
        f.write(f"F1 Score:  {f1_score:.4f}\n")
        f.write(f"IoU:       {iou:.4f}\n")
        f.write(f"TP: {tp}, FP: {fp}, FN: {fn}, TN: {tn}\n")
    
    print(f"Results saved to {prediction_folder}")

if __name__ == "__main__":
    # Parse arguments
    parser = argparse.ArgumentParser(description='Run image-to-image prediction')
    parser.add_argument('--problem', type=str, default='single_env_goal',
                        help='Problem name (default: single_env_goal)')
    parser.add_argument('--method', type=str, default='pixel_wise_binary',
                        help='Method name (default: pixel_wise_binary)')
    parser.add_argument('--load_model', action='store_true',
                        help='Load a pre-trained model instead of training')
    parser.add_argument('--verbose', action='store_true',
                        help='Print detailed progress')
    parser.add_argument('--quick', action='store_true',
                        help='Run quick preview on random samples')
    parser.add_argument('--samples', type=int, default=10,
                        help='Number of samples for quick preview (default: 10)')
    args = parser.parse_args()
    
    # Validate problem and method names
    if args.problem not in problem_map:
        raise ValueError(f"Problem '{args.problem}' not found. Available problems: {list(problem_map.keys())}")
    if args.method not in method_map:
        raise ValueError(f"Method '{args.method}' not found. Available methods: {list(method_map.keys())}")
    
    # Initialize problem and method
    problem_info = problem_map[args.problem]
    problem_config = convert_yaml_to_dict(problem_info['config'])
    problem = problem_info['class'](name=args.problem, config=problem_config)
    
    method_info = method_map[args.method]
    method_config = convert_yaml_to_dict(method_info['config'])
    
    # Handle special case for device (convert string to torch.device)
    if 'device' in method_config:
        if method_config['device'] == 'cuda' and torch.cuda.is_available():
            method_config['device'] = torch.device('cuda')
        else:
            method_config['device'] = torch.device('cpu')
            
    method = method_info['class'](name=args.method, config=method_config)
    
    if args.load_model:
        method.load_model()
    else:
        method.train(problem.get_training_data(), problem.get_evaluation_data(), None, args.verbose)
    
    # Run based on mode
    if args.quick:
        run_quick_preview(problem, method, args.samples, args.load_model, args.verbose)
    else:
        run_full_evaluation(problem, method, args.load_model, args.verbose)
