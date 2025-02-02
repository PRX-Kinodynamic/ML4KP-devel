import argparse
import mujoco
import mujoco.viewer
import yaml
import numpy as np
import math
import time
import matplotlib.pyplot as plt
from collections import deque
from matplotlib.patches import Rectangle
from scipy.spatial.transform import Rotation as R
from tqdm import tqdm
import heapq
import pickle
import time
import shapely.geometry
from pathlib import Path
import os
import torch
from PIL import Image
from torchvision import transforms

transformations = transforms.Compose([
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
])
from model import FeasibilityClassifier

def quaternion_distance_symmetric(q1, q2, symmetry_rotations=2):    
        """
        Calculate the minimum distance between quaternions considering symmetry.
        Args:
            q1, q2: Quaternions to compare (array-like, 4 elements)
            symmetry_rotations: Number of symmetric rotations (e.g., 4 for 90-degree rotations)
        Returns:
            Minimum distance considering symmetry (between 0 and 1)
        """
        # Normalize the input quaternions
        q1 = R.from_quat(q1, scalar_first=True).as_quat()
        q1 = np.asarray(q1)
        q1 /= np.linalg.norm(q1)
        
        q2 = R.from_quat(q2, scalar_first=True).as_quat()
        q2 = np.asarray(q2)
        q2 /= np.linalg.norm(q2)
        
        min_dist = float('inf')
        # Create rotation around vertical axis
        for i in range(symmetry_rotations):
            angle = i * (2 * np.pi / symmetry_rotations)
            # Create quaternion for symmetric rotation
            sym_rot = R.from_euler('z', angle).as_quat()
            # Apply symmetric rotation to q2 using quaternion multiplication
            q2_sym = R.from_quat(sym_rot) * R.from_quat(q2)
            q2_sym = q2_sym.as_quat()
            # Normalize the result
            q2_sym /= np.linalg.norm(q2_sym)

            # Calculate distance
            dot_product = abs(np.dot(q1, q2_sym))
            dot_product = np.clip(dot_product, -1.0, 1.0)  # Clamp to avoid numerical issues
            dist = 1 - dot_product
            min_dist = min(min_dist, dist)


        return min_dist

def quat2euler(quat): 
    # convert quat to euler angle (roll, pitch, yaw) manually
    x, y, z, w = quat
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    pitch_y = math.atan2(t2, t1)
    t3 = +1.0 - 2.0 * (y * y + z * z)
    t4 = +2.0 * (w * z + x * y)
    yaw_z = math.atan2(t4, t3)
    return np.array([roll_x, pitch_y, yaw_z])

def generate_edge_points(pos, size, rotation):
    """Generate 3 evenly spaced points on each edge of the box (top-down view)."""
    points = []
    x, y = pos[0], pos[1]
    w, d = size[0] - 0.05 , size[1] - 0.05
    rotation = R.from_quat(rotation, scalar_first=True).as_euler('xyz')
    angle = rotation[2]
    offset = 0.1
    
    # Helper function to rotate points
    def rotate_point(px, py, center_x, center_y):
        cos_a, sin_a = np.cos(angle), np.sin(angle)
        dx, dy = px - center_x, py - center_y
        rx = center_x + dx * cos_a - dy * sin_a
        ry = center_y + dx * sin_a + dy * cos_a
        return [rx, ry]

    # Generate 3 points on each edge
    edges = [
        # Top edge
        [(x + w * (i - 1), y + d + offset) for i in range(3)],
        # Bottom edge
        [(x + w * (i - 1), y - d - offset) for i in range(3)],
        # Right edge
        [(x + w + offset, y + d * (i - 1)) for i in range(3)],
        # Left edge
        [(x - w - offset, y + d * (i - 1)) for i in range(3)]
    ]

    # Rotate points if needed
    for edge in edges:
        for point in edge:
            if angle != 0:
                rotated = rotate_point(point[0], point[1], x, y)
                points.append(rotated)
            else:
                points.append([point[0], point[1]])
    
    return points

class EdgePoints:
    def __init__(self, pos, size, quat_rot):
        self.pos = pos
        self.size = size
        self.quat_rot = quat_rot
        self._edge_points = []
        # self.neighbors = {}
        self.generate()

    def generate(self):
        self._edge_points = generate_edge_points(self.pos, self.size, self.quat_rot)
    
    def update(self, pos, quat_rot):
        self.pos = pos.copy()
        self.quat_rot = quat_rot.copy()
        self.generate()

    def get_mid_point(self, edge_point_idx):
        if edge_point_idx == 0 or edge_point_idx == 3:
            return (np.array(self._edge_points[0]) + np.array(self._edge_points[3]))/2
        elif edge_point_idx == 2 or edge_point_idx == 5:
            return (np.array(self._edge_points[2]) + np.array(self._edge_points[5]))/2
        elif edge_point_idx == 6 or edge_point_idx == 9:
            return (np.array(self._edge_points[6]) + np.array(self._edge_points[9]))/2
        elif edge_point_idx == 8 or edge_point_idx == 11:
            return (np.array(self._edge_points[8]) + np.array(self._edge_points[11]))/2
        else:
            return (np.array(self._edge_points[1]) + np.array(self._edge_points[4]))/2
    @property
    def edge_points(self):
        return self._edge_points

class Node:
    def __init__(self, state, parent=None, node_id=None):
        self.id = node_id
        self.state = state
        self.parent = parent
        self.cost_to_come = 0
        self.cost_to_go = 0
        self.children = []

class MujocoPlant:
    def __init__(self, model, file=True, viewer=False):
        if file:
            self.spec = mujoco.MjSpec.from_file(model)
            self.model = self.spec.compile()
        else:
            self.model = model
        self.data = mujoco.MjData(self.model)
        self.sim_step_size = self.model.opt.timestep
        self.viewer = None
        # options = mujoco.MjvOption()
        # mujoco.mj_set_option(options)
        options = mujoco.MjvOption()
        # turn on contact force visualizer
        mujoco.mjv_defaultOption(options)
        if viewer:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            # self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
            # self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONSTRAINT] = True
            # self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_COM] = True
        self.is_robot_found = False
        self.robot_name = "robot"   
        try:
            robot_xpos = self.data.geom(self.robot_name).xpos
            robot_rot = R.from_matrix(self.data.geom(self.robot_name).xmat.reshape(3, 3)).as_euler('xyz')
            self.is_robot_found = True
        except:
            pass
            # print("robot not found")


        self.ngeom = self.model.ngeom
        self.movable_obstacle_names = []
        self.movable_obstacle_sizes = {}
        self.movable_obstacle_ids = {}
        self.static_obstacle_names = []
        self.static_obstacle_ids = []
        self.static_obstacle_states = {}
        self.static_obstacle_sizes = {}
        self.qpos = self.data.qpos.copy()
        self.qvel = self.data.qvel.copy()

        self.edge_points = {}
        
        # Add state tracking
        self.robot_state = {
            'position': None,
            'orientation': None,
            'velocity': None
        }
        self.x_bounds = [100, -100]
        self.y_bounds = [100, -100]
        self.obstacle_states = {}
        mujoco.mj_kinematics(self.model, self.data)
        self.update_states()  # Initialize states
        self._setup()

    def _setup(self):
        x_min, x_max = 2, -2
        y_min, y_max = 2, -2
        # x_min, x_max = self.x_bounds
        # y_min, y_max = self.y_bounds
        for i in range(self.ngeom):
            geom = self.model.geom(i)
            if geom.name == "robot":
                continue
            
            if 'movable' in geom.name:
                self.movable_obstacle_names.append(geom.name)
                self.movable_obstacle_ids[geom.name] = i    
                self.movable_obstacle_sizes[geom.name] = geom.size.copy()
                self.edge_points[geom.name] = EdgePoints(geom.pos.copy(), geom.size.copy(), geom.quat.copy())
            if 'static' in geom.name or 'wall' in geom.name:
                if 'wall' in geom.name:
                    x, y = geom.pos[0], geom.pos[1]
                    if x < x_min:
                        x_min = x
                    if x > x_max:
                        x_max = x
                    if y < y_min:
                        y_min = y
                    if y > y_max:
                        y_max = y

                self.static_obstacle_names.append(geom.name)
                self.static_obstacle_ids.append(i)
                self.static_obstacle_sizes[geom.name] = geom.size.copy()

        self.x_bounds = [x_min, x_max]
        self.y_bounds = [y_min, y_max]

        if self.is_robot_found:
            self.robot_rel_pos = self.data.geom(self.robot_name).xpos.copy()
        
        for obs_name in self.static_obstacle_names:
            self.static_obstacle_states[obs_name] = {}
            self.static_obstacle_states[obs_name]['position'] = self.data.geom(obs_name).xpos.copy()
            self.static_obstacle_states[obs_name]['orientation'] = R.from_matrix(self.data.geom(obs_name).xmat.reshape(3, 3)).as_quat(scalar_first=True)

    def _load_model(self, model_xml):
        return mujoco.MjModel.from_xml_path(model_xml)

    def update_states(self):
        """Update internal state tracking for robot and obstacles"""
        # Update robot state
        if self.is_robot_found:
            robot_geom = self.data.geom(self.robot_name)
            self.robot_state['position'] = robot_geom.xpos.copy()
            self.robot_state['orientation'] = R.from_matrix(
                robot_geom.xmat.reshape(3, 3)).as_euler('xyz')
            self.robot_state['velocity'] = self.data.qvel[:2].copy()  # Assuming 2D motion

        # Update movable obstacles
        for obs_name in self.movable_obstacle_names:
            obs_geom = self.data.geom(obs_name)
            
            self.obstacle_states[obs_name] = {
                'position': obs_geom.xpos.copy(),
                'orientation': R.from_matrix(obs_geom.xmat.copy().reshape(3, 3)).as_quat(scalar_first=True)
            }
            # Update edge points
            self.edge_points[obs_name].update(
                obs_geom.xpos.copy(), 
                R.from_matrix(obs_geom.xmat.copy().reshape(3, 3)).as_quat(scalar_first=True)
            )

            # print('obs_geom.xpos: ', obs_name, obs_geom.xpos, R.from_matrix(obs_geom.xmat.reshape(3, 3)).as_euler('xyz'), self.edge_points[obs_name].edge_points)

        self.qpos[:] = self.data.qpos.copy()
        self.qvel[:] = self.data.qvel.copy()

    def step(self, control_input, control_steps=1, viewer=False):
        
        """Step the simulation with given control input"""
        self.data.ctrl[:] = control_input
        for _ in range(control_steps):
            mujoco.mj_step(self.model, self.data)
            self.update_states()
            
            if self.viewer and viewer:
                self.viewer.sync()

    def get_robot_state(self):
        """Return current robot state"""
        return self.robot_state

    def get_obstacle_states(self):
        """Return current obstacle states"""
        return self.obstacle_states
    
    def get_qpos(self):
        return self.qpos.copy()
    
    def get_qvel(self):
        return self.qvel.copy()

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model_xml', type=str, default=None)
    # parser.add_argument('--model_folder', type=str, default=None)
    parser.add_argument('--headless', action='store_true')
    args = parser.parse_args()
    return args

def check_robot_collision(model, edge_point_idx, obstacle_name, qpos):
    # model.step(control_input, 1)
    control_input = np.zeros(model.model.nu)
    model.step(control_input, 1, viewer=True)
    # print('edge_point: ', model.edge_points['obstacle_1_movable'].edge_points[edge_point_idx])
    # print('qpos: ', model.data.qpos, qpos)
    model.data.qpos[:] = qpos
    model.data.qvel[:] = 0.0
    model.step(control_input, 1, viewer=True)
    edge_point = model.edge_points[obstacle_name].edge_points[edge_point_idx]
    # print('after edge_point: ', edge_point)
    # print("edge_point: ", edge_point)
    model.data.qpos[:2] = edge_point - model.robot_rel_pos[:2]
    model.data.qvel[:] = 0.0
    model.step(control_input, 1, viewer=True)
    mujoco.mj_kinematics(model.model, model.data)

    collision_bodies = [*model.static_obstacle_names, *model.movable_obstacle_names]
    collision_bodies.remove(obstacle_name)
    
    ncon = model.data.ncon
    # print("ncon: ", ncon)
    for i in range(ncon):
        body1_name = model.model.geom(model.data.contact[i].geom1).name
        body2_name = model.model.geom(model.data.contact[i].geom2).name
        # print("body1_name: ", body1_name)
        # print("body2_name: ", body2_name)
        if body1_name == "robot" or body2_name == "robot":
            if body1_name in collision_bodies or body2_name in collision_bodies:
                # print("collision detected for obstacle: ", body1_name, body2_name)
                return True
    return False

def check_obstacle_collision(model, obstacle_name, obstacle_pose, rel_pose):

    control_input = np.zeros(model.model.nu) 
    model.step(control_input, 1)
    
    model.data.qpos[:2] = [-100, -100]
    
    new_qpos = np.zeros((3, 1))
    new_qpos[:2, 0] = (np.array(obstacle_pose[:2]) - np.array(rel_pose[:2]))
    
    # angle = R.from_euler('xyz', [0, 0, np.arctan2(new_qpos[1], new_qpos[0])]).as_matrix()
    

    rot1 = R.from_quat(obstacle_pose[2:6], scalar_first=True).as_matrix()
    rot2 = R.from_quat(rel_pose[2:6], scalar_first=True).as_matrix()
    rot_vec = rot1 @ rot2.T
    # print(rot_vec.shape)

    rot_vec = rot_vec @ new_qpos

    # print(rot_vec.shape)

    # print(R.from_quat(quat, scalar_first=True).as_euler('xyz')[2])

    model.data.qpos[2:4] = rot_vec[:2, 0]

    model.data.qpos[5:9] = R.from_matrix(rot1 @ rot2.T).as_quat(scalar_first=True)

    model.step(control_input, 1, viewer=True)


    # model.data.geom(obstacle_name).xpos[:2] = obstacle_pose[:2]
    # model.data.geom(obstacle_name).xmat = R.from_quat(obstacle_pose[2:6], scalar_first=True).as_matrix().flatten()
    # mujoco.mj_kinematics(model.model, model.data)

    # model.step(control_input, 1, viewer=True)

    # input('before rot')
    # quat = R.from_euler('xyz', [0, 0, obstacle_pos[2] - rel_pos[2]]).as_quat(scalar_first=True)
    # model.data.qpos[5:9] = quat # this is the problem because the rotation is centered at rel_pose and not the obstacle's center
    # model.data.qvel[:] = 0.0

    # model.step(control_input, 1, viewer=True)
    # # mujoco.mj_kinematics(model.model, model.data)
    # input('after rot')

    # 
    # print(obstacle_pose[:2] - rel_pose[:2])

    # model.step(control_input, 1, viewer=True)
    # input('after rel')

    collision_bodies = [*model.static_obstacle_names]
    ncon = model.data.ncon
    for i in range(ncon):
        body1_name = model.model.geom(model.data.contact[i].geom1).name
        body2_name = model.model.geom(model.data.contact[i].geom2).name
        if (body1_name == obstacle_name and body2_name in collision_bodies) or (body2_name == obstacle_name and body1_name in collision_bodies):
            # print("collision detected for obstacle: ", body1_name, body2_name)
            return True
    return False

def check_pushing(model, obstacle_name, edge_point_idx, push_steps=500, control_steps=50, scaling=0.2):
    
    
    # model.step(control_input, 1)
    edge_point = model.edge_points[obstacle_name].edge_points[edge_point_idx]
    model.data.qpos[:2] = edge_point - model.robot_rel_pos[:2]
    model.data.qvel[:] = 0.0
    control_input = np.zeros(model.model.nu) 
    model.step(control_input, 1)

    # input()

    states = []

    obstacle_state = model.get_obstacle_states()[obstacle_name]
    obstacle_pos = obstacle_state['position']
    obstacle_quat = obstacle_state['orientation']
    qpos = model.data.qpos.copy()
    p = qpos[2:5]
    q = qpos[5:9]
    # states.append([obstacle_pos[0], obstacle_pos[1], *obstacle_quat, *p, *q, edge_point_idx, 0])
    tt = 0
    ctr = 0
    for i in range(push_steps):
        ctr += 1
        robot_state = model.get_robot_state()
        robot_pos = robot_state['position']
    
        com = model.edge_points[obstacle_name].get_mid_point(edge_point_idx)
        angle = np.arctan2(com[1] - robot_pos[1], com[0] - robot_pos[0])
       
        control_input[:2] = scaling * (np.array([np.cos(angle), np.sin(angle)]))
        model.step(control_input, control_steps, viewer=True)
        mujoco.mj_kinematics(model.model, model.data)
        tt += control_steps * 0.01
        obstacle_state = model.get_obstacle_states()[obstacle_name]
        qpos = model.data.qpos.copy()
        p = qpos[2:5]
        q = qpos[5:9]
        # if i >= 2:   
        state = [obstacle_state['position'][0], obstacle_state['position'][1], *obstacle_state['orientation'], *p, *q, edge_point_idx, ctr, control_steps, scaling]
        states.append(state)
    return states

def preprocess_edge_points(planner_model, obstacle_name, push_steps, control_steps, scaling):
    control_input = np.zeros(planner_model.model.nu)
    edge_idxs = np.arange(12)
    edge_points = {}
    for edge_idx in edge_idxs:
        edge_points[edge_idx] = []
        mujoco.mj_resetData(planner_model.model, planner_model.data)
        control_input[:] = 0.0
        planner_model.step(control_input, 1)
        states = check_pushing(planner_model, obstacle_name, edge_idx, push_steps=push_steps, control_steps=control_steps, scaling=scaling)
        edge_points[edge_idx] = np.array(states).copy()

    if planner_model.viewer is not None:
        planner_model.viewer.close()
    # with open('edge_points.pkl', 'wb') as f:
    #     pickle.dump(edge_points, f)
    mujoco.mj_resetData(planner_model.model, planner_model.data)
    planner_model.step(control_input, 1)
    return edge_points

def set_goal(model, obstacle_name=None, goal_obstacle_pos=None, goal_obstacle_quat=None, main=False, show=True):

    geom_id = 0
    rgba = [0, 1, 0, 0.25]
    if not main:
        geom_id = 1
        rgba = [1, 0, 0, 0.25]
    if goal_obstacle_pos is not None:
        if show and model.viewer is not None:
            model.viewer.user_scn.ngeom = geom_id
            mujoco.mjv_initGeom(model.viewer.user_scn.geoms[geom_id], type=mujoco.mjtGeom.mjGEOM_BOX, size=model.movable_obstacle_sizes[obstacle_name], pos=[goal_obstacle_pos[0], goal_obstacle_pos[1], 0.2], mat=R.from_quat(goal_obstacle_quat, scalar_first=True).as_matrix().flatten(), rgba=rgba)
            model.viewer.user_scn.ngeom = geom_id+1
            model.viewer.sync()
        return None, None
        
    obstacle_states = model.get_obstacle_states()
    obstacle_pos = obstacle_states[obstacle_name]['position']
    obstacle_quat = obstacle_states[obstacle_name]['orientation']
    obstacle_size = model.movable_obstacle_sizes[obstacle_name]
    diag_size = (((obstacle_size[0] * 2) ** 2 + (obstacle_size[1]*2) ** 2) ** (1/2))
    x_min, x_max = model.x_bounds
    y_min, y_max = model.y_bounds
    # print(x_min, x_max, y_min, y_max, diag_size)
    while True:
        goal_obstacle_pos = obstacle_pos.copy()
        # randomly sample a goal position
        goal_obstacle_pos[0] = np.random.uniform(x_min + diag_size/2, x_max - diag_size/2) # np.random.uniform(-2, 2)
        goal_obstacle_pos[1] = np.random.uniform(y_min + diag_size/2, y_max - diag_size/2) # np.random.uniform(-2, 2)
        # goal_obstacle_pos[0] += np.random.uniform(-1, 1)
        # goal_obstacle_pos[1] += np.random.uniform(-1, 1)

        goal_obstacle_rot = np.random.uniform(-np.pi, np.pi, 3)
        goal_obstacle_rot[:2] = 0.0
        goal_obstacle_quat = R.from_euler('xyz', goal_obstacle_rot).as_quat(scalar_first=True)

        displacement = np.linalg.norm(np.array(goal_obstacle_pos) - np.array(obstacle_pos)) + 1.0 * quaternion_distance_symmetric(goal_obstacle_quat, obstacle_quat, symmetry_rotations=2)
        
        if displacement < 2:
            break
    
    if show and model.viewer is not None:
        model.viewer.user_scn.ngeom = 1
        mujoco.mjv_initGeom(model.viewer.user_scn.geoms[geom_id], type=mujoco.mjtGeom.mjGEOM_BOX, size=model.movable_obstacle_sizes[obstacle_name], pos=[goal_obstacle_pos[0], goal_obstacle_pos[1], goal_obstacle_pos[2]/2], mat=R.from_quat(goal_obstacle_quat, scalar_first=True).as_matrix().flatten(), rgba=rgba)
        model.viewer.user_scn.ngeom = geom_id+1
        model.viewer.sync()

    return goal_obstacle_pos, goal_obstacle_quat

class Node:
    def __init__(self, state, parent=None, node_id=None):
        self.state = state
        self.parent = parent
        self.node_id = node_id
        self.cost_to_come = 0
        self.cost_to_go = 0

def check_intersection(o1_state, o2_state, o1_size, o2_size):
    """Check if two obstacles intersect, considering SE(2) transformations.
    
    Args:
        o1_state: [x, y, quat] for first obstacle
        o2_state: [x, y, quat] for second obstacle
        o1_size: [length, width] of first obstacle
        o2_size: [length, width] of second obstacle
    
    Returns:
        bool: True if obstacles intersect, False otherwise
    """
    # Extract positions and rotations
    o1_pos = o1_state[:2]
    o2_pos = o2_state[:2]
    
    # Get rotation angles from quaternions
    o1_rot = R.from_quat(o1_state[2:6], scalar_first=True).as_euler('xyz')[2]
    o2_rot = R.from_quat(o2_state[2:6], scalar_first=True).as_euler('xyz')[2]
    
    # Create corner points for each obstacle (before rotation/translation)
    def get_corner_points(size):
        l, w = size[0], size[1]
        return [[-l, -w], [-l, w], [l, w], [l, -w]]
    
    o1_corners = get_corner_points(o1_size)
    o2_corners = get_corner_points(o2_size)
    
    # Apply rotation and translation to corner points
    def transform_points(points, pos, angle):
        rot_mat = np.array([[np.cos(angle), -np.sin(angle)],
                           [np.sin(angle), np.cos(angle)]])
        transformed = []
        for p in points:
            # Rotate
            p_rot = rot_mat @ np.array(p)
            # Translate
            p_final = p_rot + pos
            transformed.append(p_final)
        return transformed
    
    o1_transformed = transform_points(o1_corners, o1_pos, o1_rot)
    o2_transformed = transform_points(o2_corners, o2_pos, o2_rot)
    
    # Create Shapely polygons and check intersection
    o1_polygon = shapely.geometry.Polygon(o1_transformed)
    o2_polygon = shapely.geometry.Polygon(o2_transformed)
    
    return o1_polygon.intersects(o2_polygon)
    

def build_motion_primitives(execution_model, obstacle_name=None, push_steps=20, control_steps=500, scaling=0.5):
    motion_primitives  = {}
    obstacles = execution_model.movable_obstacle_names
    if obstacle_name is not None:
        obstacles = [obstacle_name]
    for obstacle_name in tqdm(obstacles):
        name = execution_model.model.geom(execution_model.movable_obstacle_ids[obstacle_name]).name
        friction = execution_model.model.geom(execution_model.movable_obstacle_ids[obstacle_name]).friction
        size = execution_model.model.geom(execution_model.movable_obstacle_ids[obstacle_name]).size

        spec = mujoco.MjSpec.from_file("/home/dhruv/2024/projects/ml4kp_ktamp/resources/models/mujoco_envs_search_push/env_config_base.xml")
        body = spec.worldbody.add_body(name=obstacle_name)
        body.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_BOX, size=size, pos=[0, 0, 0.2], euler=[0, 0, 0], mass=0.1, friction=friction, condim=4)
        body.add_joint(type=mujoco.mjtJoint.mjJNT_FREE)
        preprocessing_model = MujocoPlant(spec.compile(), file=False, viewer=False)
        edge_points = preprocess_edge_points(preprocessing_model, obstacle_name, push_steps=push_steps, control_steps=control_steps, scaling=scaling)
        if preprocessing_model.viewer is not None:
            preprocessing_model.viewer.close()
        motion_primitives[obstacle_name] = edge_points.copy()
    return motion_primitives


def load_feasibility_classifier(device='cuda:0'):
    model = FeasibilityClassifier()
    model.load_state_dict(torch.load('executables/feasibility_classifier/feasibility_classifier_1.pth'))
    model.to(device)
    model.eval()
    return model

def get_feasibility_prediction(model, device='cuda:0'):
    img = Image.open('tmp.png')
    img = img.convert('RGB')
    img = transformations(img)
    img = (2 * img.unsqueeze(0)) - 1
    
    img = img.to(device)
    
    with torch.no_grad():
        pred = model(img)
    return pred

def generate_topdown_map(model, obstacle_name, goal_obstacle_pos, goal_obstacle_quat):
    fig, ax = plt.subplots(1, 1)    
    for obs_name in model.static_obstacle_names:
        obs_pos = model.static_obstacle_states[obs_name]['position']
        obs_quat = model.static_obstacle_states[obs_name]['orientation']
        obs_rot = R.from_quat(obs_quat, scalar_first=True).as_euler('xyz')[2]
        obs_size = model.static_obstacle_sizes[obs_name]
        obs_patch = Rectangle((obs_pos[0] - obs_size[0], obs_pos[1] - obs_size[1]), obs_size[0]*2, obs_size[1]*2, angle=obs_rot * (180/np.pi), rotation_point='center', color='red')
        ax.add_patch(obs_patch)

    obs_rects = []
    goal_rect = []

    for obs_name in model.movable_obstacle_names:
        color = 'yellow'
        if obs_name == obstacle_name:
            obs_pos = goal_obstacle_pos
            obs_quat = goal_obstacle_quat
            obs_size = model.movable_obstacle_sizes[obs_name]
            obs_rot = R.from_quat(obs_quat, scalar_first=True).as_euler('xyz')[2]
            rect = Rectangle((obs_pos[0] - obs_size[0], obs_pos[1] - obs_size[1]), obs_size[0]*2, obs_size[1]*2, angle=obs_rot * (180/np.pi), rotation_point='center', color='green', alpha=0.3)
            goal_rect.append(rect)
            color = 'blue'
        obs_pos = model.get_obstacle_states()[obs_name]['position']
        obs_quat = model.get_obstacle_states()[obs_name]['orientation']
        obs_size = model.movable_obstacle_sizes[obs_name]
        obs_rot = R.from_quat(obs_quat, scalar_first=True).as_euler('xyz')[2]
        obs_rect = Rectangle((obs_pos[0] - obs_size[0], obs_pos[1] - obs_size[1]), obs_size[0]*2, obs_size[1]*2, angle=obs_rot * (180/np.pi), rotation_point='center', color=color)
        obs_rects.append(obs_rect)

    for rect in obs_rects:
        ax.add_patch(rect)
    for rect in goal_rect:
        ax.add_patch(rect)
    
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    ax.axis('off')
    plt.tight_layout()
    plt.savefig(f'tmp.png', bbox_inches='tight', pad_inches=0)
    plt.close()

class TaskNode:
    def __init__(self, obstacle_name, goal_pos=None, goal_quat=None, parent=None, qpos=None, main=False, depth=0):
        self.obstacle_name = obstacle_name
        self.goal_pos = goal_pos
        self.goal_quat = goal_quat
        self.parent = parent
        self.qpos = qpos
        self.main = main
        self.depth = depth

    def set_goal(self, goal_pos, goal_quat):
        self.goal_pos = goal_pos
        self.goal_quat = goal_quat

def main():
    # exit()
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    feasibility_classifier = load_feasibility_classifier(device=device)

    is_build_motion_primitives = False
    push_steps = 20 # discretization steps
    control_steps = 500 # duration
    scaling = 0.5
    args = parse_args()
    if args.model_xml is None:
        print("Model XML not provided")
        exit()

    primitives_folder = "resources/motion_primitives"
    data_folder = "/media/dhruv/a7519aee-b272-44ae-a117-1f1ea1796db6/2024/NAMO/execution_dataset/3"
    if not os.path.exists(data_folder):
        os.makedirs(data_folder)

    existing_file_count = len(os.listdir(data_folder))
    
    if not os.path.exists(primitives_folder):
        os.makedirs(primitives_folder)

    model_no = Path(args.model_xml).stem.split('_')[-1]
    headless = args.headless
    execution_model = MujocoPlant(args.model_xml, viewer=not headless)
    # obstacle_name = np.random.choice(execution_model.movable_obstacle_names)
    control_input = np.zeros(execution_model.model.nu)
    execution_model.step(control_input, 1)

    if not os.path.exists(f'{primitives_folder}/motion_primitives_{model_no}.pkl'):
        # preprocess the environment to get the motion primitives for each movable obstacle.
        motion_primitives_coarse = build_motion_primitives(execution_model, obstacle_name=None, push_steps=push_steps, control_steps=control_steps, scaling=scaling)
        with open(f'{primitives_folder}/motion_primitives_{model_no}.pkl', 'wb') as f:
            pickle.dump(motion_primitives_coarse, f)
    else:
        with open(f'{primitives_folder}/motion_primitives_{model_no}.pkl', 'rb') as f:
            motion_primitives_coarse = pickle.load(f)

    # with open('motion_primitives_fine.pkl', 'rb') as f:
    #     motion_primitives_fine = pickle.load(f)
    
    # spec = mujoco.MjSpec.from_file("/home/dhruv/2024/projects/ml4kp_ktamp/resources/models/mujoco_envs_search_push/env_config_compile.xml")
    # body = spec.worldbody.add_body(name=obstacle_name)
    # body.add_geom(name=name, condim=4, type=mujoco.mjtGeom.mjGEOM_BOX, size=size, pos=pos, euler=R.from_quat(quat, scalar_first=True).as_euler('xyz', degrees=True), mass=0.1, friction=friction)
    # body.add_freejoint()
    
    # model = spec.compile()
    # planning_model = MujocoPlant(model, file=False, viewer=True)
    # planning_model.step(np.zeros_like(planning_model.model.nu), 1)
        
     # Update the distance function
    distance_func = lambda x, y: (
        np.linalg.norm(x[0][:2] - y[0][:2]) + 1.0 * quaternion_distance_symmetric(x[1], y[1], symmetry_rotations=2)
    )

    # Update the goal check
    goal_check = lambda x, y: (
        np.linalg.norm(x[0][:2] - y[0][:2]) < 0.05 and
        (1.0 * quaternion_distance_symmetric(x[1], y[1], symmetry_rotations=2)) < 0.05
    )

    success_count = 0
    execution_success_count = 0
    execution_failure_count = 0
    trials =  np.arange(10)
    time_limit_per_trial = 1
    replanning_count_max = 50
    planning_failures = 0
    planning_success = 0


    execution_data = []
    task_obstacle_name = np.random.choice(execution_model.movable_obstacle_names)
    qpos = execution_model.data.qpos.copy()
    task_goal_pos, task_goal_quat = set_goal(execution_model, obstacle_name=task_obstacle_name, main=True)
    task_stack = []
    goal_task = TaskNode(obstacle_name=task_obstacle_name, goal_pos=task_goal_pos, goal_quat=task_goal_quat, parent=None, qpos=qpos, main=True)
    task_idxs = np.arange(len(execution_model.movable_obstacle_names))
    np.random.shuffle(task_idxs)

    for child_task_idx in task_idxs:
        obstacle_name = execution_model.movable_obstacle_names[child_task_idx]
        # print(obstacle_name)
        obstacle_goal_pos, obstacle_goal_quat = set_goal(execution_model, obstacle_name=obstacle_name, show=False)
        obstacle_task = TaskNode(obstacle_name=obstacle_name, qpos=qpos, main=False)
        task_stack.append(obstacle_task)

    task_stack.append(goal_task)

    current_task = task_stack.pop()

    distance_change = []
    total_time_taken = 0
    
    while len(task_stack) > -1:
        
        if current_task.goal_pos is None:
            goal_pos, goal_quat = set_goal(execution_model, obstacle_name=current_task.obstacle_name, show=False)
            current_task.set_goal(goal_pos=goal_pos, goal_quat=goal_quat)
         
        set_goal(execution_model, obstacle_name=current_task.obstacle_name, goal_obstacle_pos=current_task.goal_pos, goal_obstacle_quat=current_task.goal_quat, main=current_task.main, show=True)
        goal_obstacle_pos = current_task.goal_pos
        goal_obstacle_quat = current_task.goal_quat
        obstacle_name = current_task.obstacle_name
        current_depth = current_task.depth

        mujoco.mj_resetData(execution_model.model, execution_model.data)
        execution_model.data.qpos[:] =  current_task.qpos
        execution_model.data.qvel[:] = 0
        control_input = np.zeros(execution_model.model.nu) 
        execution_model.step(control_input, 1, viewer=True)

        generate_topdown_map(execution_model, obstacle_name=obstacle_name, goal_obstacle_pos=goal_obstacle_pos, goal_obstacle_quat=goal_obstacle_quat)
        feasibility_prediction = get_feasibility_prediction(feasibility_classifier, device=device)
        
        print("feasibility_prediction: ", feasibility_prediction)
        input()
        if feasibility_prediction < 0.7:
            print("predicted not feasible")
            current_task = task_stack.pop()
            continue

        avoid_edge_points = []
        replanning_count = 0
        trajectory_data = []

        # if current_task.main and execution_success:
        #     break

        start_time = time.time()
        while True:
            check_distance_change = False
            replanning_count += 1

            dist_change = np.linalg.norm(execution_model.get_obstacle_states()[obstacle_name]['position'][:2] - goal_obstacle_pos[:2])
            if len(distance_change) >= 10:
                check_distance_change = True
                distance_change.pop(0)
            distance_change.append(dist_change)
            if replanning_count > replanning_count_max or (check_distance_change and np.array(distance_change).std() < 0.05):
                total_time_taken += time.time() - start_time
                data = {
                    'model_no': model_no,
                    'trajectory': trajectory_data,
                    'execution_success': False,
                    'obstacle_name': obstacle_name,
                    'goal_obstacle_pos': goal_obstacle_pos,
                    'goal_obstacle_quat': goal_obstacle_quat,
                    'states': execution_model.get_obstacle_states(),
                    'movable_obstacle_sizes': execution_model.movable_obstacle_sizes,
                    'static_obstacle_states': execution_model.static_obstacle_states,
                    'static_obstacle_sizes': execution_model.static_obstacle_sizes,
                    'time_taken': total_time_taken
                }
                execution_data.append(data)
                execution_failure_count += 1

                # if not current_task.main and current_depth < 3:
                #     task_idxs = np.arange(len(execution_model.movable_obstacle_names))
                #     np.random.shuffle(task_idxs)
                #     for child_task_idx in task_idxs:
                #         name = execution_model.movable_obstacle_names[child_task_idx]
                #         # goal_pos, goal_quat = set_goal(execution_model, obstacle_name=name)
                #         obstacle_task = TaskNode(obstacle_name=name, qpos=current_task.qpos, main=False, depth=current_depth+1)
                #         # obstacle_task.set_goal(goal_pos=goal_pos, goal_quat=goal_quat)
                #         task_stack.append(obstacle_task)
                #     goal_task = TaskNode(obstacle_name=task_obstacle_name, goal_pos=task_goal_pos, goal_quat=task_goal_quat, parent=None, qpos=qpos, main=True)
                    # task_stack.append(goal_task)
                break
            
            state = execution_model.get_obstacle_states()

            pos = execution_model.get_obstacle_states()[obstacle_name]['position']
            quat = execution_model.get_obstacle_states()[obstacle_name]['orientation']

            trajectory_data.append(state.copy())

            # spec = mujoco.MjSpec.from_file("/home/dhruv/2024/projects/ml4kp_ktamp/resources/models/mujoco_envs_search_push/env_config_compile.xml")
            # body = spec.worldbody.add_body(name=obstacle_name)
            # body.add_geom(name=name, condim=4, type=mujoco.mjtGeom.mjGEOM_BOX, size=size, pos=pos, euler=R.from_quat(quat, scalar_first=True).as_euler('xyz', degrees=True), mass=0.1, friction=friction)
            # body.add_freejoint()
            
            # model = spec.compile()
            # planning_model = MujocoPlant(model, file=False, viewer=False)
            # planning_model.step(np.zeros_like(planning_model.model.nu), 1)

            # mujoco.mj_resetData(planning_model.model, planning_model.data)
            # set_goal(planning_model, obstacle_name=obstacle_name, goal_obstacle_pos=goal_obstacle_pos, goal_obstacle_quat=goal_obstacle_quat)
            # control_input = np.zeros(planning_model.model.nu)
            # planning_model.step(control_input, control_steps)


            # obstacle_state = planning_model.get_obstacle_states()[obstacle_name]
            init_obstacle_pos = pos[:2].copy()
            init_obstacle_quat = quat.copy()

            # qpos = planning_model.data.qpos.copy()

            state = [init_obstacle_pos[0], init_obstacle_pos[1], *init_obstacle_quat, None, None, None]
            current_node = Node(state=state, parent=None, node_id=0)

            current_node.cost_to_go = distance_func((init_obstacle_pos, init_obstacle_quat), (goal_obstacle_pos, goal_obstacle_quat))

            rotation = R.from_quat(init_obstacle_quat, scalar_first=True).as_euler('xyz')

            mat = np.array([[np.cos(rotation[2]), -np.sin(rotation[2])], [np.sin(rotation[2]), np.cos(rotation[2])]])

            queue = []
            ctr = 1
            goal_reached = False
            
            
            edge_points = motion_primitives_coarse[obstacle_name]
            
            for edge_idx in range(len(edge_points)):
                if edge_idx in avoid_edge_points:
                    continue
                edge_points_transformed = edge_points[edge_idx].copy()
                edge_points_transformed[:, :2] = (mat @ edge_points_transformed[:, :2].T).T + state[:2]
                
                rot = R.from_quat(state[2:6], scalar_first=True).as_matrix()
                batch_rot = R.from_quat(edge_points_transformed[:, 2:6], scalar_first=True).as_matrix()
                rotated_quat = R.from_matrix(batch_rot @ rot).as_quat(scalar_first=True)
                edge_points_transformed[:, 2:6] = rotated_quat

                break_out = False
                for point in edge_points_transformed:
                    qpos = None # planning_model.data.qpos.copy()
                    point_state = [*point[:6], qpos, *point[-4:]]
                    # check if point is in collisionsa
                    node = Node(state=point_state, parent=current_node, node_id=ctr)
                    node.cost_to_come = 0 # current_node.cost_to_come + distance_func((np.array(point)[:2], R.from_euler('xyz', [0, 0, np.array(point)[2]]).as_quat(scalar_first=True)), (init_obstacle_pos[:2], init_obstacle_quat))
                    node.cost_to_go = distance_func((np.array(point)[:2], np.array(point)[2:6]), (goal_obstacle_pos, goal_obstacle_quat))
                    heapq.heappush(queue, (node.cost_to_go + node.cost_to_come + np.random.uniform(0, 0.0001), node))
                    ctr += 1
            
            closest_node = (None, float('inf'))
            planning_time_start = time.time()
            while len(queue) > 0:
                _, current_node = heapq.heappop(queue)
                rot_mat = R.from_quat(current_node.state[2:6], scalar_first=True).as_matrix()
                current_edge_idx = int(current_node.state[-2])
                
                # if planning_model.viewer is not None:
                #     planning_model.viewer.user_scn.ngeom = 1
                #     mujoco.mjv_initGeom(planning_model.viewer.user_scn.geoms[1], type=mujoco.mjtGeom.mjGEOM_BOX, size=planning_model.movable_obstacle_sizes[obstacle_name], pos=[current_node.state[0], current_node.state[1], current_node.state[3]/2], mat=rot_mat.flatten(), rgba=[1, 1, 1, 0.25])
                #     planning_model.viewer.user_scn.ngeom = 2
                #     planning_model.viewer.sync()
                
                obstacle_pos = current_node.state[:2]
                obstacle_quat = current_node.state[2:6]
                obstacle_rot = R.from_quat(obstacle_quat, scalar_first=True).as_euler('xyz')[2]

                # goal check
                # print(distance_func((obstacle_pos, obstacle_quat), (goal_obstacle_pos, goal_obstacle_quat)))
                dist = distance_func((obstacle_pos, obstacle_quat), (goal_obstacle_pos, goal_obstacle_quat))
                if dist < closest_node[1]:
                    closest_node = (current_node, distance_func((obstacle_pos, obstacle_quat), (goal_obstacle_pos, goal_obstacle_quat)))

                
                edge_points = motion_primitives_coarse[obstacle_name]
                    # print("Using coarse motion primitives")
                if goal_check((obstacle_pos, obstacle_quat), (goal_obstacle_pos, goal_obstacle_quat)):
                    closest_node = (current_node, distance_func((obstacle_pos, obstacle_quat), (goal_obstacle_pos, goal_obstacle_quat)))
                    goal_reached = True
                    break

                if (time.time() - planning_time_start) > time_limit_per_trial:
                    break

                mat = np.array([[np.cos(obstacle_rot), -np.sin(obstacle_rot)], [np.sin(obstacle_rot), np.cos(obstacle_rot)]])
                
                for edge_idx in edge_points:
                    edge_points_transformed = edge_points[edge_idx].copy()
                    edge_points_transformed[:, :2] = (mat @ (edge_points_transformed[:, :2]).T).T + obstacle_pos
                    batch_rot = R.from_quat(edge_points_transformed[:, 2:6], scalar_first=True).as_matrix()
                    rot = R.from_quat(obstacle_quat, scalar_first=True).as_matrix()
                    rotated_quat = R.from_matrix(batch_rot @ rot).as_quat(scalar_first=True)
                    edge_points_transformed[:, 2:6] = rotated_quat
                    extra_cost = 0

                    for point in edge_points_transformed:
                        qpos = None # planning_model.data.qpos.copy()
                        point_state = [*point[:6], qpos, *point[-4:]]
                        # collision check before adding to queue
                        node = Node(state=point_state, parent=current_node, node_id=ctr)
                        node.cost_to_go = (distance_func((np.array(point)[:2], np.array(point)[2:6]), (goal_obstacle_pos, goal_obstacle_quat)) + extra_cost)
                        node.cost_to_come = 0 # current_node.cost_to_come + distance_func((np.array(point)[:2], np.array(point)[2:6]), (obstacle_pos, obstacle_quat))
                        heapq.heappush(queue, (node.cost_to_go + node.cost_to_come, node))
                        ctr += 1

            # if planning_model.viewer is not None:
            #     planning_model.viewer.close()
            # mujoco.mj_deleteModel(planning_model.model)
            # mujoco.mj_deleteData(planning_model.data)

            if not goal_reached:
                planning_failures += 1
                # print("trial no:", trial)
            else:
                planning_success += 1
                
                # print("planning failed")
            # continue
            trajectory = []
            edge_points_taken = []
            traj_push_steps = []
            traj_scaling = []
            traj_control_steps = []
            node = closest_node[0]
            while True:
                trajectory.append(node.state[:3])
                if node.parent is None:
                    break
                edge_points_taken.append(int(node.state[-4]))
                traj_push_steps.append(int(node.state[-3]))
                traj_control_steps.append(int(node.state[-2]))
                traj_scaling.append(node.state[-1])
                node = node.parent
                
            edge_points_taken = edge_points_taken[::-1]
            # push_steps = push_steps[::-1]
            trajectory = trajectory[::-1]
            traj_push_steps = traj_push_steps[::-1]
            traj_control_steps = traj_control_steps[::-1]
            traj_scaling = traj_scaling[::-1]

            # print(traj_push_steps, traj_control_steps, traj_scaling)
            # input()

            execution_success = False
            for i in range(len(edge_points_taken)):
                avoid_edge_points.append(edge_points_taken[i])
                # input()
                if check_robot_collision(execution_model, edge_points_taken[i], obstacle_name, execution_model.data.qpos.copy()):
                    break
                
                check_pushing(execution_model, obstacle_name, edge_points_taken[i], push_steps=traj_push_steps[i], control_steps=traj_control_steps[i], scaling=traj_scaling[i])
                state = execution_model.get_obstacle_states()

                trajectory_data.append(state.copy())
                obstacle_state = state[obstacle_name]

                if np.linalg.norm(np.array(trajectory[i+1][:2]) - np.array(obstacle_state['position'][:2])) > 0.1:
                    break
                
                if goal_check((obstacle_state['position'], obstacle_state['orientation']), (goal_obstacle_pos, goal_obstacle_quat)):
                    execution_success = True
                    break
                avoid_edge_points = []
           
            if not execution_success:
                continue
            else:
                total_time_taken += time.time() - start_time
                data = {
                    'model_no': model_no,
                    'trajectory': trajectory_data,
                    'execution_success': True,
                    'obstacle_name': obstacle_name,
                    'goal_obstacle_pos': goal_obstacle_pos,
                    'goal_obstacle_quat': goal_obstacle_quat,
                    'states': execution_model.get_obstacle_states(),
                    'movable_obstacle_sizes': execution_model.movable_obstacle_sizes,
                    'static_obstacle_states': execution_model.static_obstacle_states,
                    'static_obstacle_sizes': execution_model.static_obstacle_sizes,
                    'time_taken': total_time_taken
                }

                execution_data.append(data)

                execution_success_count += 1

                if not current_task.main and current_depth < 3:
                    qpos = execution_model.data.qpos.copy()
                    task_idxs = np.arange(len(execution_model.movable_obstacle_names))
                    np.random.shuffle(task_idxs)
                    for child_task_idx in task_idxs:
                        name = execution_model.movable_obstacle_names[child_task_idx]
                        obstacle_goal_pos, obstacle_goal_quat = set_goal(execution_model, obstacle_name=name)
                        obstacle_task = TaskNode(obstacle_name=name, qpos=qpos, main=False, depth=current_depth+1)
                        obstacle_task.set_goal(goal_pos=obstacle_goal_pos, goal_quat=obstacle_goal_quat)
                        task_stack.append(obstacle_task)
                    goal_task = TaskNode(obstacle_name=task_obstacle_name, goal_pos=task_goal_pos, goal_quat=task_goal_quat, parent=None, qpos=qpos, main=True, depth=current_depth+1)
                    task_stack.append(goal_task)
                break
        
        if current_task.main and execution_success:
            break
        if len(task_stack) == 0:
            break
        current_task = task_stack.pop()

    # print("Success rate: ", success_count / len(trials))
    # print("Execution success rate: ", execution_success_count / (execution_failure_count + execution_success_count))

    if execution_model.viewer is not None:
        execution_model.viewer.close()
        
    with open(f'{data_folder}/execution_data_{existing_file_count}.pkl', 'wb') as f:
        pickle.dump(execution_data, f)

if __name__ == "__main__":
    main()