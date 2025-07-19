import subprocess
import os
import time
import yaml
import uuid
import argparse
import multiprocessing
import signal
import sys
import traceback
import shutil
import random
import logging
import mujoco
from datetime import datetime
from tqdm import tqdm
import glob
import json
from collections import defaultdict
'''
run with: python executables/utils/collect_namo_data_mp.py --xml-path resources/models/custom_walled_envs/empty/env_config_1.xml --iterations 1000 --output-dir /common/users/dm1487/namo_data/env_config_1
run with: python executables/utils/collect_namo_data_mp.py --one-env --xml-path resources/models/custom_walled_envs/apr18_25/random_start_fixed_goal_one_env_1 --iterations 2 --output-dir /common/users/dm1487/namo_data/apr19/random_start_fixed_goal_one_env_1_v2

run with: python executables/utils/collect_namo_data_mp.py --xml-path resources/models/custom_walled_envs/apr27_25/random_start_fixed_goal_many_env_config_2 --iterations 25 --output-dir /common/users/dm1487/namo_data/apr27/random_start_fixed_goal_many_env_config_2 --one-env

run with: python executables/utils/collect_namo_data_mp.py --xml-path resources/models/custom_walled_envs/jun22/random_start_random_goal_single_obstacle_room_2_200k_halfrad --iterations 10 --output-dir /common/users/dm1487/namo_data/jun22/random_start_random_goal_single_obstacle_room_2_200k --start 26666 --end 40000
'''

# Global variables for cleanup
temp_config_dir = None
config_files = []

def setup_temp_directory():
    """Create a temporary directory for config files"""
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    temp_dir = f"resources/input_files/temp_namo_configs_{timestamp}"
    os.makedirs(temp_dir, exist_ok=True)
    return temp_dir

def cleanup(output_dir=None):
    """Clean up temporary config files and directory, and optionally iteration attempts"""
    global temp_config_dir, config_files
    print("\nCleaning up temporary files...")
    
    # Remove individual config files
    for config_path in config_files:
        if os.path.exists(config_path):
            try:
                os.remove(config_path)
            except Exception as e:
                print(f"Warning: Could not remove {config_path}: {e}")
    
    # Remove temporary directory if it exists
    if temp_config_dir and os.path.exists(temp_config_dir):
        try:
            shutil.rmtree(temp_config_dir)
        except Exception as e:
            print(f"Warning: Could not remove temporary directory {temp_config_dir}: {e}")
    
    # Remove iteration attempts directory if output_dir is provided
    if output_dir:
        attempts_dir = os.path.join(output_dir, "iteration_attempts")
        if os.path.exists(attempts_dir):
            try:
                shutil.rmtree(attempts_dir)
                print(f"Removed iteration attempts directory: {attempts_dir}")
            except Exception as e:
                print(f"Warning: Could not remove iteration attempts directory {attempts_dir}: {e}")
    
    print("Cleanup complete")

def signal_handler(sig, frame):
    """Handle keyboard interrupt (Ctrl+C)"""
    print("\nKeyboard interrupt detected. Stopping processes and cleaning up...")
    cleanup()
    sys.exit(1)

def track_iteration_attempt(output_dir, env_id, iteration_id, batch_id):
    """Track that we attempted an iteration for an environment"""
    attempts_dir = os.path.join(output_dir, "iteration_attempts")
    os.makedirs(attempts_dir, exist_ok=True)
    
    # Create a simple tracking file for this attempt
    attempt_file = os.path.join(attempts_dir, f"{env_id}_iter_{iteration_id}_{batch_id}.txt")
    with open(attempt_file, 'w') as f:
        f.write(f"Environment: {env_id}\n")
        f.write(f"Iteration: {iteration_id}\n")
        f.write(f"Batch: {batch_id}\n")
        f.write(f"Timestamp: {datetime.now().isoformat()}\n")

def count_existing_sequences_per_env(output_dir):
    """Count sequences and iterations for each environment configuration"""
    env_sequence_counts = defaultdict(int)
    env_iteration_counts = defaultdict(int)
    env_single_no_action = defaultdict(bool)  # Track environments with single datapoint, no action
    
    # Count sequences from JSON files and check for single datapoint with no action
    pattern = os.path.join(output_dir, "sequence_*.json")
    sequence_files = glob.glob(pattern)
    
    for file_path in sequence_files:
        if 'final_state' not in file_path:
            continue
        try:
            with open(file_path, 'r') as f:
                data = json.load(f)
                config_name = data.get('config_name', '')
                if config_name:
                    env_sequence_counts[config_name] += 1
                    
                    # Check if this sequence has single datapoint with no action
                    data_points = data.get('data_points', [])
                    if len(data_points) == 1 and 'action' not in data_points[0]:
                        env_single_no_action[config_name] = True
                        
        except (json.JSONDecodeError, KeyError, FileNotFoundError) as e:
            print(f"Warning: Could not process sequence file {file_path}: {e}")
            continue
    
    # Count iterations from attempt tracking files
    attempts_dir = os.path.join(output_dir, "iteration_attempts")
    if os.path.exists(attempts_dir):
        attempt_files = glob.glob(os.path.join(attempts_dir, "*.txt"))
        
        for file_path in attempt_files:
            try:
                filename = os.path.basename(file_path)
                # Format: {env_id}_iter_{iteration_id}_{batch_id}.txt
                env_id = filename.split('_iter_')[0]
                env_iteration_counts[env_id] += 1
            except Exception as e:
                print(f"Warning: Could not process attempt file {file_path}: {e}")
                continue
    
    return dict(env_sequence_counts), dict(env_iteration_counts), dict(env_single_no_action)

def filter_environments_needing_data(xml_paths, output_dir, target_sequences=30, max_iterations_per_env=50):
    """Filter environments that still need more sequences and haven't exceeded iteration limits"""
    env_counts, iter_counts, single_no_action = count_existing_sequences_per_env(output_dir)
    
    environments_needing_data = []
    
    for xml_path in xml_paths:
        env_id = os.path.splitext(os.path.basename(xml_path))[0]
        current_sequences = env_counts.get(env_id, 0)
        current_iterations = iter_counts.get(env_id, 0)
        has_single_no_action = single_no_action.get(env_id, False)
        
        # Skip environment if it has a sequence with single datapoint and no action
        if has_single_no_action:
            print(f"Skipping {env_id}: has sequence with single datapoint and no action key")
            continue
        
        # Only include if:
        # 1. Haven't reached target sequences AND
        # 2. Haven't exceeded maximum iterations
        if current_sequences < target_sequences and current_iterations < max_iterations_per_env:
            environments_needing_data.append({
                'xml_path': xml_path,
                'env_id': env_id,
                'current_sequences': current_sequences,
                'current_iterations': current_iterations,
                'sequences_needed': target_sequences - current_sequences,
                'iterations_remaining': max_iterations_per_env - current_iterations
            })
    
    return environments_needing_data

def generate_configs_batch_aggressive(base_config_path, batch_size, output_dir, environments_needing_data, 
                                    temp_dir, batch_num, one_env=True, max_iterations_per_env=50):
    """Generate configuration files that aggressively fill the entire batch"""
    global config_files
    
    config_paths = []
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    batch_id = f"{timestamp}_batch{batch_num}_{str(uuid.uuid4())[:6]}"
    
    if not environments_needing_data:
        return []
    
    configs_generated = 0
    round_number = 0
    
    print(f"Filling batch of {batch_size} configs across {len(environments_needing_data)} environments...")
    
    # Keep cycling through environments until batch is full
    while configs_generated < batch_size:
        envs_used_this_round = 0
        
        for env_info in environments_needing_data[:1]:
            if configs_generated >= batch_size:
                break
                
            xml_path = env_info['xml_path']
            env_id = env_info['env_id']
            current_iterations = env_info['current_iterations']
            sequences_needed = env_info['sequences_needed']
            iterations_remaining = env_info['iterations_remaining']
            
            # Skip if this environment has reached its iteration limit
            if current_iterations + round_number >= max_iterations_per_env:
                continue
            
            try:
                # Load base configuration
                with open(base_config_path, 'r') as f:
                    base_config = yaml.safe_load(f)
                
                # Get robot goal from XML
                model = mujoco.MjModel.from_xml_path(xml_path)
                data = mujoco.MjData(model)
                site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'goal')
                if site_id != -1:
                    robot_goal = model.site_pos[site_id][:2].tolist()
                else:
                    robot_goal = [2.5, 2.5]  # default
                del model, data
                
                # Generate config
                process_config = base_config.copy()
                process_config['xml_path'] = '/'.join(xml_path.split('/')[2:])
                
                if 'data_collection' not in process_config:
                    process_config['data_collection'] = {}
                    
                process_config['smoothing_enabled'] = True
                process_config['total_iter'] = 10
                process_config['data_collection']['enabled'] = True
                process_config['data_collection']['output_dir'] = output_dir
                
                # Calculate actual iteration number for this attempt
                iteration_id = current_iterations + round_number
                process_config['data_collection']['run_id'] = f"{batch_id}_{env_id}_iter{iteration_id}"
                process_config['robot_goal'] = robot_goal
                process_config['random_seed'] = random.randint(1, 1000000)
                process_config['object_strategy'] = 0
                process_config['visualize'] = False
                
                # Write config to file
                config_path = os.path.join(temp_dir, f"config_batch{batch_num}_{env_id}_iter{iteration_id}_round{round_number}.yaml")
                with open(config_path, 'w') as f:
                    yaml.dump(process_config, f, default_flow_style=False)
                
                # TRACK THIS ITERATION ATTEMPT IMMEDIATELY
                track_iteration_attempt(output_dir, env_id, iteration_id, batch_id)
                
                config_paths.append('/'.join(config_path.split("/")[2:]))
                configs_generated += 1
                envs_used_this_round += 1
                
            except Exception as e:
                print(f"Warning: Could not generate config for {env_id}: {e}")
                continue
        
        if envs_used_this_round == 0:
            print(f"Warning: Could not generate configs for any environment in round {round_number}")
            break
            
        round_number += 1
        
        if round_number > 100:
            print(f"Warning: Reached maximum rounds ({round_number}), stopping config generation")
            break
    
    # Report distribution (fix the env_id extraction)
    env_config_counts = {}
    for config_path in config_paths:
        filename = config_path.split('/')[-1]
        # Format: config_batch{N}_{env_id}_iter{N}_round{N}.yaml
        # Extract env_id properly (it might contain underscores)
        parts = filename.split('_')
        
        # Find batch and iter positions
        batch_idx = -1
        iter_idx = -1
        for i, part in enumerate(parts):
            if part.startswith('batch'):
                batch_idx = i
            elif part.startswith('iter'):
                iter_idx = i
                break
        
        if batch_idx != -1 and iter_idx != -1:
            env_id = '_'.join(parts[batch_idx + 1:iter_idx])
            env_config_counts[env_id] = env_config_counts.get(env_id, 0) + 1
    
    print(f"Generated {configs_generated} configs distributed as:")
    for env_id, count in sorted(env_config_counts.items()):
        print(f"  {env_id}: {count} configs")
    
    config_files.extend(config_paths)
    return config_paths

def setup_logging(log_dir):
    """Set up logging configuration for the main process"""
    os.makedirs(log_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(log_dir, f"namo_collection_main_{timestamp}.log")
    
    # Configure logging for the main process
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(log_file),
            logging.StreamHandler()  # Main script output still goes to console
        ]
    )
    
    logging.info(f"Main process logging initialized. Log file: {log_file}")
    return log_file, timestamp

def run_parallel_data_collection(executable_path, config_paths, log_dir, timestamp, max_processes=None):
    """Run multiple processes in parallel with a process pool"""
    if max_processes is None:
        max_processes = max(1, multiprocessing.cpu_count() - 1)  # Leave one CPU free
    
    logging.info(f"Running {len(config_paths)} iterations using {max_processes} parallel processes")
    
    random.shuffle(config_paths)
    # Create a logs subdirectory for this batch
    batch_log_dir = os.path.join(log_dir, f"batch_{timestamp}")
    os.makedirs(batch_log_dir, exist_ok=True)
    logging.info(f"Process logs will be stored in: {batch_log_dir}")
    
    # Initialize a list to track all processes
    active_processes = []
    completed = 0
    failed = 0
    
    try:
        # Start initial batch of processes
        for i, config_path in enumerate(config_paths[:max_processes]):
            # Create a unique log file for this iteration
            iter_log_file = os.path.join(batch_log_dir, f"iter_{i+1:04d}.log")
            
            # Start the process with its output redirected to its own log file
            cmd = [executable_path, config_path]
            logging.info(f"Starting iteration {i+1}/{len(config_paths)} - Log: {os.path.basename(iter_log_file)}")
            
            with open(iter_log_file, 'w') as f:
                # Write a header to the log file
                f.write(f"=== Process output for iteration {i+1}/{len(config_paths)} ===\n")
                f.write(f"Command: {' '.join(cmd)}\n")
                f.write(f"Started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                f.write("="*80 + "\n\n")
                f.flush()
                
                # Start process with output redirected to this log file
                proc = subprocess.Popen(
                    cmd, 
                    stdout=f,
                    stderr=f,
                    universal_newlines=True
                )
                
                active_processes.append((proc, config_path, i+1, iter_log_file))
        
        # Process management loop
        remaining_configs = config_paths[max_processes:]
        config_idx = max_processes
        
        while active_processes:
            # Check for completed processes
            for i, (proc, config_path, task_num, log_path) in enumerate(active_processes[:]):
                if proc.poll() is not None:  # Process has completed
                    active_processes.remove((proc, config_path, task_num, log_path))
                    
                    # Append completion status to the process log file
                    with open(log_path, 'a') as f:
                        f.write(f"\n\n{'='*80}\n")
                        f.write(f"Process completed at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                        f.write(f"Return code: {proc.returncode}\n")
                        if proc.returncode == 0:
                            f.write("Status: SUCCESS\n")
                        else:
                            f.write("Status: FAILED\n")
                        f.write(f"{'='*80}\n")
                    
                    # Log completion status to main log
                    if proc.returncode == 0:
                        logging.info(f"Iteration {task_num}/{len(config_paths)} completed successfully")
                        completed += 1
                    else:
                        logging.error(f"Iteration {task_num}/{len(config_paths)} failed with code {proc.returncode}")
                        failed += 1
                    
                    # Start a new process if there are more configs
                    if remaining_configs:
                        config_path = remaining_configs.pop(0)
                        config_idx += 1
                        
                        # Create a new log file for this iteration
                        iter_log_file = os.path.join(batch_log_dir, f"iter_{config_idx:04d}.log")
                        
                        cmd = [executable_path, config_path]
                        logging.info(f"Starting iteration {config_idx}/{len(config_paths)} - Log: {os.path.basename(iter_log_file)}")
                        
                        with open(iter_log_file, 'w') as f:
                            # Write a header to the log file
                            f.write(f"=== Process output for iteration {config_idx}/{len(config_paths)} ===\n")
                            f.write(f"Command: {' '.join(cmd)}\n")
                            f.write(f"Started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                            f.write("="*80 + "\n\n")
                            f.flush()
                            
                            # Start process with output redirected to this log file
                            proc = subprocess.Popen(
                                cmd, 
                                stdout=f,
                                stderr=f,
                                universal_newlines=True
                            )
                            
                            active_processes.append((proc, config_path, config_idx, iter_log_file))
            
            # Short sleep to avoid high CPU usage in this loop
            time.sleep(0.1)
    
    except KeyboardInterrupt:
        logging.warning("\nKeyboard interrupt detected. Terminating all running processes...")
        # Terminate all active processes
        for proc, _, _, _ in active_processes:
            try:
                proc.terminate()
            except:
                pass
        
        # Wait a bit for processes to terminate
        time.sleep(1)
        
        # Force kill if still running
        for proc, _, _, _ in active_processes:
            if proc.poll() is None:
                try:
                    proc.kill()
                except:
                    pass
        
        raise KeyboardInterrupt
    
    finally:
        # Report results using logging (will go to both console and log file)
        logging.info("\nData collection summary:")
        logging.info(f"  Completed: {completed} iterations")
        logging.info(f"  Failed: {failed} iterations")
        logging.info(f"  Total: {len(config_paths)} iterations")
        
        # Calculate success rate
        success_rate = (completed / len(config_paths)) * 100 if len(config_paths) > 0 else 0
        logging.info(f"  Success rate: {success_rate:.2f}%")
        logging.info(f"  Process logs directory: {batch_log_dir}")

def run_adaptive_data_collection(executable_path, base_config_path, output_dir, xml_paths, 
                                temp_dir, log_dir, max_processes=24, batch_size=50, 
                                target_sequences=30, max_iterations_per_env=50, one_env=True):
    """Run data collection in adaptive batches until target sequences are reached"""
    
    print(f"Starting adaptive data collection:")
    print(f"- Target: {target_sequences} sequences per environment")
    print(f"- Max iterations per environment: {max_iterations_per_env}")
    print(f"- Batch size: {batch_size} configs per batch")
    print(f"- Max parallel processes: {max_processes}")
    print(f"- Total environments: {len(xml_paths)}")
    
    batch_num = 0
    total_iterations = 0
    
    while True:
        batch_num += 1
        print(f"\n{'='*60}")
        print(f"BATCH {batch_num}")
        print(f"{'='*60}")
        
        # Check current status
        environments_needing_data = filter_environments_needing_data(
            xml_paths, output_dir, target_sequences, max_iterations_per_env)
        
        if not environments_needing_data:
            print("🎉 All environments have either reached the target sequences or iteration limits!")
            break
        
        print(f"Environments still needing data: {len(environments_needing_data)}")
        
        # Show progress for first few environments
        for i, env_info in enumerate(environments_needing_data[:5]):
            print(f"  {env_info['env_id']}: {env_info['current_sequences']}/{target_sequences} sequences "
                  f"({env_info['current_iterations']}/{max_iterations_per_env} iterations)")
        if len(environments_needing_data) > 5:
            print(f"  ... and {len(environments_needing_data) - 5} more environments")
        
        # Generate configs for this batch
        config_paths = generate_configs_batch_aggressive(
            base_config_path, batch_size, output_dir, 
            environments_needing_data, temp_dir, batch_num, one_env, max_iterations_per_env
        )
        
        if not config_paths:
            print("No more configs to generate. Stopping.")
            break
        
        print(f"Generated {len(config_paths)} configs for batch {batch_num}")
        total_iterations += len(config_paths)
        
        # Run this batch
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        setup_logging(log_dir)  # Set up logging for this batch
        
        print(f"Running batch {batch_num} with {len(config_paths)} processes...")
        run_parallel_data_collection(executable_path, config_paths, log_dir, 
                                    f"{timestamp}_batch{batch_num}", max_processes)
        
        # Brief pause between batches
        time.sleep(2)
    
    # Final summary with detailed analysis
    print(f"\n{'='*60}")
    print("FINAL SUMMARY")
    print(f"{'='*60}")
    
    final_counts, final_iterations, final_single_no_action = count_existing_sequences_per_env(output_dir)
    
    print(f"Total iterations executed: {total_iterations}")
    print(f"Total batches: {batch_num}")
    print(f"Max iterations per environment: {max_iterations_per_env}")
    print("\nPer-environment results:")
    
    # Categorize results
    successful_envs = []
    failed_envs = []
    partial_envs = []
    
    for xml_path in sorted(xml_paths, key=lambda x: os.path.basename(x)):
        env_id = os.path.splitext(os.path.basename(xml_path))[0]
        sequences = final_counts.get(env_id, 0)
        iterations = final_iterations.get(env_id, 0)
        efficiency = sequences / iterations if iterations > 0 else 0
        has_single_no_action = final_single_no_action.get(env_id, False)
        
        status = ""
        if sequences >= target_sequences:
            status = "✅ SUCCESS"
            successful_envs.append(env_id)
        elif iterations >= max_iterations_per_env:
            status = "❌ ITERATION_LIMIT"
            failed_envs.append(env_id)
        elif has_single_no_action:
            status = "❌ SINGLE_NO_ACTION"
            failed_envs.append(env_id)
        else:
            status = "⚠️  PARTIAL"
            partial_envs.append(env_id)
        
        print(f"  {env_id}: {sequences}/{target_sequences} sequences, {iterations}/{max_iterations_per_env} iterations "
              f"(efficiency: {efficiency:.2f}) {status}")
    
    # Summary statistics
    print(f"\n{'='*60}")
    print("SUMMARY STATISTICS")
    print(f"{'='*60}")
    print(f"✅ Successful environments: {len(successful_envs)}/{len(xml_paths)} ({len(successful_envs)/len(xml_paths)*100:.1f}%)")
    print(f"❌ Hit iteration limit: {len(failed_envs)}/{len(xml_paths)} ({len(failed_envs)/len(xml_paths)*100:.1f}%)")
    print(f"⚠️  Partial completion: {len(partial_envs)}/{len(xml_paths)} ({len(partial_envs)/len(xml_paths)*100:.1f}%)")
    
    if failed_envs:
        print(f"\nEnvironments that hit iteration limit ({max_iterations_per_env} iterations):")
        for env_id in failed_envs[:10]:  # Show first 10
            sequences = final_counts.get(env_id, 0)
            iterations = final_iterations.get(env_id, 0)
            efficiency = sequences / iterations if iterations > 0 else 0
            print(f"  {env_id}: {sequences}/{target_sequences} sequences (efficiency: {efficiency:.2f})")
        if len(failed_envs) > 10:
            print(f"  ... and {len(failed_envs) - 10} more")
    
    # Create summary file
    summary_file = os.path.join(output_dir, f"collection_summary_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json")
    summary_data = {
        "total_iterations": total_iterations,
        "total_batches": batch_num,
        "target_sequences_per_env": target_sequences,
        "max_iterations_per_env": max_iterations_per_env,
        "completion_timestamp": datetime.now().isoformat(),
        "summary_stats": {
            "total_environments": len(xml_paths),
            "successful_environments": len(successful_envs),
            "failed_environments": len(failed_envs),
            "partial_environments": len(partial_envs),
            "success_rate": len(successful_envs) / len(xml_paths) if xml_paths else 0
        },
        "environments": {}
    }
    
    for xml_path in xml_paths:
        env_id = os.path.splitext(os.path.basename(xml_path))[0]
        sequences = final_counts.get(env_id, 0)
        iterations = final_iterations.get(env_id, 0)
        has_single_no_action = final_single_no_action.get(env_id, False)
        
        status = "success" if sequences >= target_sequences else \
                "iteration_limit" if iterations >= max_iterations_per_env else \
                "partial"
        
        summary_data["environments"][env_id] = {
            "sequences_collected": sequences,
            "iterations_executed": iterations,
            "efficiency": sequences / iterations if iterations > 0 else 0,
            "target_reached": sequences >= target_sequences,
            "status": status,
            "has_single_no_action": has_single_no_action
        }
    
    with open(summary_file, 'w') as f:
        json.dump(summary_data, f, indent=2)
    
    print(f"\nSummary saved to: {summary_file}")
    
    # Clean up iteration attempts directory at the end
    cleanup_iteration_attempts(output_dir)

def cleanup_iteration_attempts(output_dir):
    """Clean up the iteration attempts directory after data collection is complete"""
    attempts_dir = os.path.join(output_dir, "iteration_attempts")
    if os.path.exists(attempts_dir):
        try:
            shutil.rmtree(attempts_dir)
            print(f"Cleaned up iteration attempts directory: {attempts_dir}")
        except Exception as e:
            print(f"Warning: Could not remove iteration attempts directory {attempts_dir}: {e}")

if __name__ == "__main__":
    # Set up signal handler for keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)
    
    parser = argparse.ArgumentParser(description="Run NAMO data collection in adaptive batches")
    parser.add_argument("--base-config", default="resources/input_files/examples/tasks/tamp_plan.yaml", 
                       help="Path to base YAML configuration file")
    parser.add_argument("--xml-path", required=True, 
                       help="Path to the environment XML file or directory")
    parser.add_argument("--output-dir", default="/common/users/dm1487/namo_data/adaptive_run", 
                       help="Directory to store collected data")
    parser.add_argument("--log-dir", default="/common/users/dm1487/namo_data/logs", 
                       help="Directory to store log files")
    parser.add_argument("--num-processes", type=int, default=25, 
                       help="Maximum number of parallel processes (default: 24)")
    parser.add_argument("--batch-size", type=int, default=50,
                       help="Number of configs per batch (default: 50)")
    parser.add_argument("--target-sequences", type=int, default=30,
                       help="Target number of sequences per environment (default: 30)")
    parser.add_argument("--executable", default="./bin/examples/namo_v2/interface_namo", 
                       help="Path to the interface_namo executable")
    parser.add_argument("--start", type=int, default=0,
                       help="Start index for the xml paths (default: 0)")
    parser.add_argument("--end", type=int, default=None,
                       help="End index for the xml paths (default: None)")
    parser.add_argument("--seed", type=int, default=None,
                       help="Random seed for generating run seeds (default: current time)")
    parser.add_argument("--one-env", action="store_true", 
                       help="Use one-env mode for primitive sharing")
    parser.add_argument("--max-iterations-per-env", type=int, default=300,
                       help="Maximum number of iterations per environment before giving up (default: 50)")
    args = parser.parse_args()
    
    # Set random seed for reproducibility
    if args.seed is not None:
        random.seed(args.seed)
    else:
        random.seed(int(time.time()))
    
    try:
        # Create temporary directory for config files
        temp_config_dir = setup_temp_directory()
        print(f"Using temporary directory for configs: {temp_config_dir}")
        
        # Get XML paths
        if os.path.isdir(args.xml_path):
            xml_files = [f for f in os.listdir(args.xml_path) if f.endswith('.xml')]
            xml_paths = [os.path.join(args.xml_path, f) for f in xml_files]
        else:
            xml_paths = [args.xml_path]
        
        # Sort XML paths by numeric suffix if present
        try:
            sorted_xml_paths = sorted(xml_paths, key=lambda x: int(x.split('/')[-1].split('.xml')[0].split('_')[-1]))
        except (ValueError, IndexError):
            sorted_xml_paths = sorted(xml_paths)
        
        if args.end is None:
            args.end = len(sorted_xml_paths)
        
        selected_xml_paths = sorted_xml_paths[args.start:args.end]
        
        print(f"Processing {len(selected_xml_paths)} environments (indices {args.start} to {args.end-1})")
        print(f"Target: {args.target_sequences} sequences per environment")
        
        # Make sure output directory exists
        os.makedirs(args.output_dir, exist_ok=True)
        
        # Run adaptive data collection
        run_adaptive_data_collection(
            executable_path=args.executable,
            base_config_path=args.base_config,
            output_dir=args.output_dir,
            xml_paths=selected_xml_paths,
            temp_dir=temp_config_dir,
            log_dir=args.log_dir,
            max_processes=args.num_processes,
            batch_size=args.batch_size,
            target_sequences=args.target_sequences,
            max_iterations_per_env=args.max_iterations_per_env,
            one_env=args.one_env
        )
        
        print(f"Data collection complete!")
    
    except KeyboardInterrupt:
        print("\nOperation was cancelled by user")
    
    except Exception as e:
        print(f"Error: {e}")
        traceback.print_exc()
    
    finally:
        # Clean up temporary files
        cleanup(args.output_dir)
