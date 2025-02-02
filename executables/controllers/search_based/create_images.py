import os
import pickle
import numpy as np
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as R
from glob import glob
from tqdm import tqdm
from matplotlib.patches import Rectangle
from PIL import Image
import gc
import h5py

def get_rect(pos, quat, size, color, alpha=1.0):
    angle = (R.from_quat(quat, scalar_first=True).as_euler('xyz')[2]) * 180/np.pi
    rect = Rectangle((pos[0] - size[0], pos[1] - size[1]), size[0]*2, size[1]*2, rotation_point='center', angle=angle, fill=True, color=color, alpha=alpha)
    return rect

def process_single_scene(data_entry, temp_dir='temp'):
    """Process a single scene, with explicit cleanup"""
    plt.ioff()  # Turn off interactive mode
    
    images = []
    for scene_idx in range(5):
        fig, ax = plt.subplots(1, 1, figsize=(5, 5))
        ax.set_xlim(-2, 2)
        ax.set_ylim(-2, 2)
        ax.axis('off')
        
        # Draw static obstacles
        if scene_idx in [0, 4]:
            for key, state in data_entry['static_obstacle_states'].items():
                rect = get_rect(
                    state['position'][:2],
                    state['orientation'],
                    data_entry['static_obstacle_sizes'][key],
                    'black'
                )
                ax.add_patch(rect)
        
        # Draw movable obstacles (excluding target)
        if scene_idx in [1, 4]:
            for key in data_entry['states']:
                if key != data_entry['obstacle_name']:
                    init_state = data_entry['trajectory'][0][key]
                    rect = get_rect(
                        init_state['position'][:2],
                        init_state['orientation'],
                        data_entry['movable_obstacle_sizes'][key],
                        'black'
                    )
                    ax.add_patch(rect)
        
        # Draw target object initial state
        if scene_idx in [2, 4]:  # Added scene 4 to show target in complete scene
            key = data_entry['obstacle_name']
            if key in data_entry['states']:
                init_state = data_entry['trajectory'][0][key]
                rect = get_rect(
                    init_state['position'][:2],
                    init_state['orientation'],
                    data_entry['movable_obstacle_sizes'][key],
                    'black'
                )
                ax.add_patch(rect)
        
        # Draw goal state
        if scene_idx == 3:
            key = data_entry['obstacle_name']
            if key in data_entry['states']:
                rect = get_rect(
                    data_entry['goal_obstacle_pos'][:2],
                    data_entry['goal_obstacle_quat'],
                    data_entry['movable_obstacle_sizes'][key],
                    'black'
                )
                ax.add_patch(rect)
        
        temp_path = f'{temp_dir}/scene_{scene_idx}.png'
        fig.savefig(temp_path, bbox_inches='tight', pad_inches=0)
        plt.close(fig)
        
        with Image.open(temp_path) as img:
            arr = np.array(img.convert('L'), dtype=np.float32)
            arr[arr > 0] = 1.0
            images.append(arr)
        
        os.remove(temp_path)
    
    result = np.stack(images, axis=-1)
    images.clear()
    gc.collect()
    
    return result

def process_pickle_file(pickle_path, output_dir, batch_size=50):
    """Process a single pickle file into HDF5 format"""
    success_path = os.path.join(output_dir, 'success.h5')
    failure_path = os.path.join(output_dir, 'failure.h5')
    temp_dir = os.path.join(output_dir, 'temp')
    
    os.makedirs(temp_dir, exist_ok=True)
    
    # Open both HDF5 files
    success_file = h5py.File(success_path, 'a')
    failure_file = h5py.File(failure_path, 'a')
    
    try:
        # Load pickle file
        with open(pickle_path, 'rb') as file:
            data = pickle.load(file)
        
        # Process in batches
        for batch_start in range(0, len(data), batch_size):
            batch_end = min(batch_start + batch_size, len(data))
            batch = data[batch_start:batch_end]
            
            for idx, entry in enumerate(batch):
                try:
                    scene_array = process_single_scene(entry, temp_dir)
                    
                    # Generate unique identifier
                    file_id = os.path.basename(pickle_path).split('.')[0].split('_')[-1]
                    dataset_name = f"trajectory_{file_id}_{batch_start + idx}"
                    
                    # Choose file based on success/failure
                    h5_file = success_file if entry['time_taken'] <= 30 else failure_file
                    
                    # Create dataset and add metadata
                    dset = h5_file.create_dataset(dataset_name, data=scene_array, 
                                                compression='gzip', 
                                                compression_opts=4)
                    
                    # Add metadata (converting strings to bytes for HDF5 compatibility)
                    dset.attrs['time_taken'] = float(entry['time_taken'])
                    dset.attrs['execution_success'] = bool(entry['execution_success'])
                    if 'obstacle_name' in entry:
                        dset.attrs['obstacle_name'] = entry['obstacle_name'].encode('utf-8')
                    
                    del scene_array
                    
                except Exception as e:
                    print(f"Error processing entry {batch_start + idx} from {pickle_path}: {str(e)}")
                    continue
            
            # Clear batch from memory
            del batch
            gc.collect()
        
        # Clear file data from memory
        del data
        gc.collect()
        
    finally:
        # Close HDF5 files
        success_file.close()
        failure_file.close()
        
        # Cleanup temp directory
        if os.path.exists(temp_dir):
            for file in os.listdir(temp_dir):
                os.remove(os.path.join(temp_dir, file))
            os.rmdir(temp_dir)

def process_dataset(data_folder, output_dir):
    """Process entire dataset folder"""
    total_ctr = 0
    execution_success_ctr = 0
    
    # Process each pickle file
    pickle_files = glob(f'{data_folder}/execution_data_*.pkl')
    for pickle_file in tqdm(pickle_files, desc="Processing pickle files"):
        try:
            # Count statistics
            with open(pickle_file, 'rb') as file:
                data = pickle.load(file)
            total_ctr += len(data)
            execution_success_ctr += sum(d['execution_success'] for d in data)
            del data
            
            # Process file
            process_pickle_file(pickle_file, output_dir)
            
        except Exception as e:
            print(f"Error processing file {pickle_file}: {str(e)}")
            continue
        
        gc.collect()
    
    # Print statistics
    success_rate = execution_success_ctr / total_ctr if total_ctr > 0 else 0
    print(f"Total samples: {total_ctr}")
    print(f"Success rate: {success_rate:.2%}")

if __name__ == "__main__":
    data_folder = "/media/dhruv/a7519aee-b272-44ae-a117-1f1ea1796db6/2024/NAMO/execution_dataset/2"
    output_dir = "/media/dhruv/a7519aee-b272-44ae-a117-1f1ea1796db6/2024/NAMO/test1/data_h5/0"
    
    process_dataset(data_folder, output_dir)