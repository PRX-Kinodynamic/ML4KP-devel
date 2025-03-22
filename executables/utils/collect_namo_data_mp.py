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
from datetime import datetime

# Global variables for cleanup
temp_config_dir = None
config_files = []

def setup_temp_directory():
    """Create a temporary directory for config files"""
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    temp_dir = f"resources/input_files/temp_namo_configs_{timestamp}"
    os.makedirs(temp_dir, exist_ok=True)
    return temp_dir

def cleanup():
    """Clean up temporary config files and directory"""
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
    
    print("Cleanup complete")

def signal_handler(sig, frame):
    """Handle keyboard interrupt (Ctrl+C)"""
    print("\nKeyboard interrupt detected. Stopping processes and cleaning up...")
    cleanup()
    sys.exit(1)

def generate_configs(base_config_path, num_iterations, output_dir, xml_path, temp_dir):
    """Generate configuration files for multiple iterations of a single environment"""
    global config_files
    
    # Load base configuration
    with open(base_config_path, 'r') as f:
        base_config = yaml.safe_load(f)
    
    # Create a single run ID for this batch of processes
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    batch_id = f"{timestamp}_{str(uuid.uuid4())[:6]}"
    config_paths = []
    
    # Extract environment name from XML path
    env_id = os.path.splitext(os.path.basename(xml_path))[0]
    
    # Create a config file for each iteration
    for iteration in range(num_iterations):
        # Clone the base config
        process_config = base_config.copy()
        
        # Update XML path
        process_config['xml_path'] = '/'.join(xml_path.split('/')[2:])
        
        # Ensure data collection section exists
        if 'data_collection' not in process_config:
            process_config['data_collection'] = {}
        
        # Set data collection parameters
        process_config['data_collection']['enabled'] = True
        process_config['data_collection']['output_dir'] = output_dir
        process_config['data_collection']['run_id'] = f"{batch_id}_{env_id}_iter{iteration}"
        
        # Set a unique random seed for each iteration
        process_config['random_seed'] = random.randint(1, 1000000)
        
        # Write config to file in temporary directory
        config_path = os.path.join(temp_dir, f"config_{batch_id}_iter{iteration}.yaml")

        
        with open(config_path, 'w') as f:
            yaml.dump(process_config, f, default_flow_style=False)
        config_paths.append('/'.join(config_path.split("/")[2:]))
    
    # Store config paths for cleanup
    config_files = config_paths
    
    return config_paths, batch_id

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

if __name__ == "__main__":
    # Set up signal handler for keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)
    
    parser = argparse.ArgumentParser(description="Run NAMO data collection in parallel for a single environment")
    parser.add_argument("--base-config", default="resources/input_files/examples/tasks/tamp_plan.yaml", 
                       help="Path to base YAML configuration file")
    parser.add_argument("--xml-path", required=True, 
                       help="Path to the environment XML file")
    parser.add_argument("--output-dir", default="/common/users/dm1487/namo_data/single_env_run", 
                       help="Directory to store collected data")
    parser.add_argument("--log-dir", default="/common/users/dm1487/namo_data/logs", 
                       help="Directory to store log files")
    parser.add_argument("--iterations", type=int, default=1000,
                       help="Number of iterations to run (default: 1000)")
    parser.add_argument("--num-processes", type=int, default=12, 
                       help="Maximum number of parallel processes (default: 1)")
    parser.add_argument("--executable", default="./bin/examples/namo/interface_namo", 
                       help="Path to the interface_namo executable")
    parser.add_argument("--seed", type=int, default=None,
                       help="Random seed for generating run seeds (default: current time)")
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
        
        # Verify XML file exists
        if not os.path.exists(args.xml_path):
            print(f"Error: XML file {args.xml_path} not found")
            exit(1)
        
        print(f"Using environment: {args.xml_path}")
        print(f"Will run {args.iterations} iterations")
        
        # Make sure output directory exists
        os.makedirs(args.output_dir, exist_ok=True)
        
        # Generate configs for all iterations
        configs, batch_id = generate_configs(
            args.base_config, 
            args.iterations, 
            args.output_dir, 
            args.xml_path,
            temp_config_dir
        )
        
        print(f"Generated {len(configs)} configuration files with batch ID: {batch_id}")
        
        # Set up logging - returns log file path and timestamp for naming
        main_log_file, run_timestamp = setup_logging(args.log_dir)
        
        # Run the processes
        run_parallel_data_collection(args.executable, configs, args.log_dir, run_timestamp, args.num_processes)
        
        print(f"Data collection complete for batch {batch_id}")
    
    except KeyboardInterrupt:
        print("\nOperation was cancelled by user")
    
    except Exception as e:
        print(f"Error: {e}")
        traceback.print_exc()
    
    finally:
        # Clean up temporary files
        cleanup()
