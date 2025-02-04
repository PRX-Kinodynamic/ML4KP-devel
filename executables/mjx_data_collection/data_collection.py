import mujoco
import mujoco.mjx
import jax
import jax.numpy as jnp
import time

def create_mjx_model(xml_path):
    """Create an MJX model from an XML file."""
    # First load the model in standard MuJoCo
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    
    # Convert to MJX model
    mjx_model = mujoco.mjx.put_model(model)
    mjx_data = mujoco.mjx.put_data(model,data)
    
    return mjx_model, mjx_data

def simulate_step(model, data, ctrl):
    """Perform one simulation step with MJX."""
    # Set control input
    data = data.replace(ctrl=ctrl)
    
    # Step the simulation
    data = mujoco.mjx.step(model, data)
    
    return data

@jax.jit
def run_simulation(model, data, controls, steps):
    """Run a simulation for multiple steps with MJX.
    
    Args:
        model: MJX model
        data: MJX data with shape (n_envs,) for parallel environments
        controls: Control inputs with shape (n_steps, n_envs, ctrl_size)
        steps: Number of simulation steps
    """
    def step(carry, ctrl):
        return jax.vmap(lambda d, c: simulate_step(model, d, c))(carry, ctrl), carry.qpos
    
    final_state, traj = jax.lax.scan(step, data, controls)
    return final_state, traj

def visualize_trajectory(xml_path, trajectory, fps=30):
    """Visualize the trajectory using MuJoCo's built-in viewer."""
    # Create regular MuJoCo model and data
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    
    # Create the viewer
    viewer = mujoco.viewer.launch_passive(model, data)
    
    # Playback the trajectory
    for qpos in trajectory:
        # Update the visualization state
        data.qpos = qpos
        mujoco.mj_forward(model, data)
        
        # Render and sync to desired FPS
        viewer.sync()
        time.sleep(1.0/fps)
    
    # Close the viewer
    viewer.close()

def main():
    xml_path = "resources/models/cylinder/env_config_1.xml"
    model, data = create_mjx_model(xml_path)
    
    # Parameters for parallel simulation
    n_envs = 10  # Number of parallel environments
    n_steps = 100
    ctrl_size = model.nu
    
    # Create random controls for multiple environments
    controls = jax.random.uniform(
        jax.random.PRNGKey(0),
        shape=(n_steps, n_envs, ctrl_size)  # Shape is (timesteps, n_envs, control_size)
    )
    
    # Replicate initial state for all environments
    batch_data = jax.tree_map(lambda x: jnp.repeat(x[None], n_envs, axis=0), data)
    
    # Run parallel simulation
    final_states, trajectories = run_simulation(model, batch_data, controls, n_steps)
    
    print(f"Final positions shape: {final_states.qpos.shape}")  # Shape: (n_envs, qpos_size)
    print(f"Trajectories shape: {trajectories.shape}")  # Shape: (n_steps, n_envs, qpos_size)
    
    # Visualize the first environment's trajectory
    visualize_trajectory(xml_path, trajectories[:, 0, :])

if __name__ == "__main__":
    main()
