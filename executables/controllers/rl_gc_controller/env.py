import numpy as np
import gymnasium as gym
from gymnasium import spaces
import mujoco
import yaml
import mujoco.viewer
import mediapy as media
from PIL import Image
import matplotlib.pyplot as plt

# More legible printing from numpy.
np.set_printoptions(precision=3, suppress=True, linewidth=100)

class GCRLMujocoEnv(gym.Env):
    def __init__(self, model_path, config_path, render_mode=None):
        super().__init__()
        
        # Load the MuJoCo model
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)


         

        # id = self.model.geom('robot').id
        # mujoco.mj_kinematics(self.model, self.data)
        # print(self.data.geom('robot').xpos)

        
        # duration, framerate = 4.0, 30
        # frames = []
        # mujoco.mj_resetData(self.model, self.data)

        # with mujoco.Renderer(self.model) as renderer:
        #     while self.data.time < duration:
        #         mujoco.mj_step(self.model, self.data)
        #         if len(frames) < self.data.time * framerate:
        #             renderer.update_scene(self.data)
        #             img = renderer.render()
        #             frames.append(img)
        
        # media.write_video("test.mp4", frames, fps=framerate)
        # exit()
        
        # Load config file
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Get environment boundaries from config
        self.env_size = self.config['env_size']
        
        mujoco.mj_kinematics(self.model, self.data)
        self.robot_init_pos = self.data.geom('robot').xpos[:2].copy()
        # print(self.robot_init_pos)


        self.x_bounds = [-self.env_size[0]/2 + 0.1, self.env_size[0]/2 - 0.1]
        self.y_bounds = [-self.env_size[1]/2 + 0.1, self.env_size[1]/2 - 0.1]
        
        # Define action space (2D continuous actions for x and y movement)
        self.action_space = spaces.Box(
            low=-1,
            high=1,
            shape=(2,),
            dtype=np.float32
        )
        
        # Define observation space using environment boundaries
        self.observation_space = spaces.Dict({
            'observation': spaces.Box(
                low=np.array([self.x_bounds[0], self.y_bounds[0]], dtype=np.float32),
                high=np.array([self.x_bounds[1], self.y_bounds[1]], dtype=np.float32),
                dtype=np.float32
            ),
            'achieved_goal': spaces.Box(
                low=np.array([self.x_bounds[0], self.y_bounds[0]], dtype=np.float32),
                high=np.array([self.x_bounds[1], self.y_bounds[1]], dtype=np.float32),
                dtype=np.float32
            ),
            'desired_goal': spaces.Box(
                low=np.array([self.x_bounds[0], self.y_bounds[0]], dtype=np.float32),
                high=np.array([self.x_bounds[1], self.y_bounds[1]], dtype=np.float32),
                dtype=np.float32
            ),
        })
        
        # Initialize goal
        self.goal = None
        
        # Viewer setup
        self.render_mode = render_mode
        self.viewer = None
        
        # Add max episode steps
        self.max_episode_steps = 1000  # You can adjust this value
        self.current_step = 0
    
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        # Reset simulation
        mujoco.mj_resetData(self.model, self.data)
        
        # Sample random initial position for robot within bounds
        init_x = self.np_random.uniform(self.x_bounds[0], self.x_bounds[1])
        init_y = self.np_random.uniform(self.y_bounds[0], self.y_bounds[1])

        self.data.qpos[:2] = np.array([init_x, init_y]) - self.robot_init_pos 
        self.data.qvel[:] = 0.0

        # Sample random goal relative to robot's initial position
        self.goal = self.sample_goal()
        
        # Forward the simulation
        mujoco.mj_forward(self.model, self.data)

        
        observation = self._get_obs()
        info = {}
        
        if self.render_mode == "human":
            self.render()
        
        # Reset step counter
        self.current_step = 0
        
        return observation, info
    
    def step(self, action):
        # Increment step counter
        self.current_step += 1

        
        # Apply action
        self.data.ctrl[:] = action
        mujoco.mj_step(self.model, self.data)
        
        # Get observation
        observation = self._get_obs()
        
        # Calculate reward
        reward = self.compute_reward(
            observation['achieved_goal'],
            observation['desired_goal'],
            None
        )
        
        # Check if done
        terminated = self._is_success(observation['achieved_goal'], observation['desired_goal'])
        # Add truncation condition based on max steps
        truncated = self.current_step >= self.max_episode_steps
        
        info = {
            'is_success': terminated,
            'TimeLimit.truncated': truncated
        }
        
        if self.render_mode == "human":
            self.render()
        
        return observation, reward, terminated, truncated, info
    
    def _get_obs(self):
        # Get robot position relative to robot's initial position
        robot_pos = self.data.geom('robot').xpos[:2]
        return {
            'observation': robot_pos.astype(np.float32),
            'achieved_goal': robot_pos.astype(np.float32),
            'desired_goal': self.goal.astype(np.float32)
        }

    def sample_goal(self):
        # Sample random goal position within the environment bounds
        # These bounds are already relative to robot's initial position
        # mujoco.mj_kinematics(self.model, self.data)
        # goal_x = x + np.random.uniform(-0.25, 0.25)
        # goal_y = y + np.random.uniform(-0.25, 0.25)

        goal_x = self.np_random.uniform(self.x_bounds[0], self.x_bounds[1])
        goal_y = self.np_random.uniform(self.y_bounds[0], self.y_bounds[1])
        return np.array([goal_x, goal_y], dtype=np.float32)
    
    def compute_reward(self, achieved_goal, desired_goal, info):
        # Sparse reward
        # away from goal, -0.01 reward
        # close to goal, 1.0 reward
        scale = -1 * 1/self.max_episode_steps
        _distance = np.linalg.norm(achieved_goal - desired_goal, axis=-1).reshape(-1, 1)

        distance = (_distance.copy() > 0.1) * scale
        distance[_distance < 0.1] = 10.0

        if distance.shape[0] == 1:
            distance = distance[0][0]

        return distance
    
    def _is_success(self, achieved_goal, desired_goal):
        distance = np.linalg.norm(achieved_goal - desired_goal)
        return distance < 0.1
    
    def render(self):
        if self.viewer is None and self.render_mode == "human":
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            # self.viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_WIREFRAME] = 1
            self.viewer.user_scn.ngeom = 0
            mujoco.mjv_initGeom(self.viewer.user_scn.geoms[0], type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.1, 0.1, 0.1], pos=[self.goal[0], self.goal[1], 0], mat=np.eye(3).flatten(), rgba=[0, 1, 0, 0.25])
            self.viewer.user_scn.ngeom = 1
            self.viewer.sync()
            
        if self.viewer is not None:
            mujoco.mjv_initGeom(self.viewer.user_scn.geoms[0], type=mujoco.mjtGeom.mjGEOM_SPHERE, size=[0.1, 0.1, 0.1], pos=[self.goal[0], self.goal[1], 0], mat=np.eye(3).flatten(), rgba=[0, 1, 0, 0.25])
            self.viewer.user_scn.ngeom = 1
            self.viewer.sync()
    
    def close(self):
        if self.viewer is not None:
            self.viewer.close()
            self.viewer = None
    

if __name__ == "__main__":
    import time
    # Example usage
    env = GCRLMujocoEnv(
        "resources/models/mujoco_envs/env_config_1.xml",
        "resources/input_files/mujoco_envs/env_configs/env_config_1.yaml",
        render_mode="human"
)

    # Test the environment
    obs, info = env.reset()
    
    start_time = time.time()
    for i in range(200000):
        action = env.action_space.sample()
        action *= 0.0

        # action[1] = -0.01
        obs, reward, terminated, truncated, info = env.step(action)
        # time.sleep(0.01)
        # print(obs['observation'])
        if (i+1) % 10000 == 0 or terminated or truncated:
            time.sleep(1.0)
            obs, info = env.reset()

        # print(f"Time taken: {time.time() - start_time} seconds")
        # start_time = time.time()
        # print(obs['observation'], obs['achieved_goal'], obs['desired_goal'])

    env.close()


    