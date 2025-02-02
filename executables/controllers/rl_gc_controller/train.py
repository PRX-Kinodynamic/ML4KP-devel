from stable_baselines3 import HerReplayBuffer, SAC, TD3, DDPG
from stable_baselines3.common.callbacks import CheckpointCallback
from typing import Optional
import os
import torch

from env import GCRLMujocoEnv

def create_env(model_path: str, config_path: str) -> GCRLMujocoEnv:
    return GCRLMujocoEnv(model_path=model_path, config_path=config_path, render_mode="none")

def create_model(env: GCRLMujocoEnv, load_path: Optional[str] = None) -> SAC:
    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    print(f"Using device: {device}")
    
    if load_path and os.path.exists(load_path):
        return SAC.load(load_path, env=env, device=device)
    
    # Set learning_starts to be at least one episode length
    learning_starts = env.max_episode_steps
    
    return SAC(
        "MultiInputPolicy",
        env,
        replay_buffer_class=HerReplayBuffer,
        verbose=1,
        buffer_size=100_000,
        learning_rate=1e-3,
        gamma=0.95,
        batch_size=256,
        policy_kwargs=dict(net_arch=[256, 256]),
        device=device,
        learning_starts=learning_starts,  # Now using the episode length
        replay_buffer_kwargs=dict(
            n_sampled_goal=4,
            goal_selection_strategy="future",
        ),
    )

def train(model: SAC, total_timesteps: int, save_path: Optional[str] = None):
    # Add checkpoint callback if save_path is provided
    callbacks = []
    if save_path:
        checkpoint_callback = CheckpointCallback(
            save_freq=100000,
            save_path=save_path,
            name_prefix="sac_model"
        )
        callbacks.append(checkpoint_callback)
    
    model.learn(total_timesteps=total_timesteps, callback=callbacks)
    
    if save_path:
        final_model_path = os.path.join(save_path, "final_model")
        model.save(final_model_path)
        print(f"Model saved to {final_model_path}")

def main(model_path: str, model_config_path: str, total_timesteps: int, save_path: Optional[str] = None):
    
    # Create environment
    env = create_env(model_path, model_config_path)
    
    # Create or load model
    model = create_model(env)
    
    # Train model
    os.makedirs(save_path, exist_ok=True)
    train(model, total_timesteps=total_timesteps, save_path=save_path)

if __name__ == "__main__":
    model_path = "resources/models/mujoco_envs/env_config_1.xml"
    model_config_path = "resources/input_files/mujoco_envs/env_configs/env_config_1.yaml"

    main(model_path, model_config_path, total_timesteps=1_000_000, save_path="saved_models")