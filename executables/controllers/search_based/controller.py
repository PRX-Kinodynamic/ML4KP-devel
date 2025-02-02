import argparse
import mujoco
import mujoco.viewer
import yaml
import numpy as np
import math
import time
import matplotlib.pyplot as plt
from collections import deque
from scipy.spatial.transform import Rotation as R
import heapq

np.random.seed(42)


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
    def __init__(self, pos, size, euler_rot):
        self.pos = pos
        self.size = size
        self.euler_rot = euler_rot
        self._edge_points = []
        self.generate()

    def generate(self):
        self._edge_points = generate_edge_points(self.pos, self.size, self.euler_rot)
    
    def update(self, pos, euler_rot):
        self.pos = pos
        self.euler_rot = euler_rot
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
    def __init__(self, model_xml, viewer=False ):
        self.model = self._load_model(model_xml)
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
        self.robot_name = "robot"   

        robot_xpos = self.data.geom(self.robot_name).xpos
        robot_rot = R.from_matrix(self.data.geom(self.robot_name).xmat.reshape(3, 3)).as_euler('xyz')

        self.ngeom = self.model.ngeom
        self.movable_obstacle_names = []
        self.movable_obstacle_sizes = []
        self.movable_obstacle_ids = []
        self.static_obstacle_names = []
        self.static_obstacle_ids = []
        self.qpos = self.data.qpos.copy()
        self.qvel = self.data.qvel.copy()

        self.edge_points = {}
        
        # Add state tracking
        self.robot_state = {
            'position': None,
            'orientation': None,
            'velocity': None
        }
        self.obstacle_states = {}
        self.update_states()  # Initialize states

        self._setup()

    def _setup(self):
        for i in range(self.ngeom):
            geom = self.model.geom(i)
            if 'movable' in geom.name:
                self.movable_obstacle_names.append(geom.name)
                self.movable_obstacle_ids.append(i)
                self.movable_obstacle_sizes.append(geom.size)
                self.edge_points[geom.name] = EdgePoints(geom.pos, geom.size, geom.quat)
            if 'static' in geom.name or 'wall' in geom.name:
                self.static_obstacle_names.append(geom.name)
                self.static_obstacle_ids.append(i)

        self.robot_rel_pos = self.data.geom(self.robot_name).xpos.copy()

    def _load_model(self, model_xml):
        return mujoco.MjModel.from_xml_path(model_xml)

    def update_states(self):
        """Update internal state tracking for robot and obstacles"""
        # Update robot state
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
                'orientation': R.from_matrix(obs_geom.xmat.reshape(3, 3)).as_euler('xyz')
            }
            # Update edge points
            self.edge_points[obs_name].update(
                obs_geom.xpos, 
                R.from_matrix(obs_geom.xmat.reshape(3, 3)).as_euler('xyz')
            )

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
    parser.add_argument('--model_xml', type=str, required=True)
    args = parser.parse_args()
    return args


def check_pushing(execution_model):

    steps = 0
    total_control_steps = 10000
    control_steps = 500
    

    control_input = np.zeros(execution_model.model.nu) 
    execution_model.step(control_input, control_steps)

    edge_point_idx = 0
    edge_point = execution_model.edge_points['obstacle_1_movable'].edge_points[edge_point_idx]
    execution_model.data.qpos[:2] = edge_point - execution_model.robot_rel_pos[:2]
    execution_model.step(control_input, control_steps)
    qpos, qvel = execution_model.get_qpos(), execution_model.get_qvel()
    execution_model.viewer.sync()

    ctr = 0

    while execution_model.viewer.is_running():
        execution_model.data.qvel[:] = 0.0
        control_input[:] = 0.0
        execution_model.step(control_input, 1)

        robot_state = execution_model.get_robot_state()
        robot_pos = robot_state['position']
        
        com = execution_model.edge_points['obstacle_1_movable'].get_mid_point(edge_point_idx)
        angle = np.arctan2(com[1] - robot_pos[1], com[0] - robot_pos[0])
        control_input[:2] = 0.2 * np.array([np.cos(angle), np.sin(angle)])
        execution_model.step(control_input, control_steps)
        ctr += 1
        if ctr > 1000:
            break
    return

def main():
    args = parse_args()
    execution_model = MujocoPlant(args.model_xml, viewer=True)
    # oscillation
    # check_pushing(execution_model)
    # exit()
    steps = 0
    total_control_steps = 10000
    control_steps = 500
    

    control_input = np.zeros(execution_model.model.nu) 
    execution_model.step(control_input, control_steps)

    print(execution_model.movable_obstacle_names)
    # exit()
    

    edge_point_idx = 0
    edge_point = execution_model.edge_points['obstacle_1_movable'].edge_points[edge_point_idx]
    execution_model.data.qpos[:2] = edge_point - execution_model.robot_rel_pos[:2]
    execution_model.step(control_input, control_steps)
    qpos, qvel = execution_model.get_qpos(), execution_model.get_qvel()
    execution_model.viewer.sync()

    obstacle_states = execution_model.get_obstacle_states()
    obstacle_pos = obstacle_states['obstacle_1_movable']['position']
    obstacle_rot = obstacle_states['obstacle_1_movable']['orientation']
    obstacle_quat = R.from_euler('xyz', obstacle_rot).as_quat()
    
    # goal_obstacle_pos = np.array([-0.4850799, 1.49981665, 0.19998862])
    # goal_obstacle_quat = np.array([0.0, 0.0, 0.50502225, 0.86310633])

    while True:     
        # set goal obstacle position and orientation
        goal_obstacle_pos = obstacle_pos.copy()
        goal_obstacle_rot = np.random.uniform(-np.pi, np.pi, 3)
        goal_obstacle_rot[:2] = 0.0
        goal_obstacle_quat = R.from_euler('xyz', goal_obstacle_rot).as_quat()

        # randomly sample a goal position
        goal_obstacle_pos[0] += np.random.uniform(-1, 1)
        goal_obstacle_pos[1] += np.random.uniform(-1, 1)

        execution_model.viewer.user_scn.ngeom = 0
        mujoco.mjv_initGeom(execution_model.viewer.user_scn.geoms[0], type=mujoco.mjtGeom.mjGEOM_BOX, size=execution_model.movable_obstacle_sizes[0], pos=[goal_obstacle_pos[0], goal_obstacle_pos[1], goal_obstacle_pos[2]/2], mat=R.from_quat(goal_obstacle_quat).as_matrix().flatten(), rgba=[0, 1, 0, 0.25])
        execution_model.viewer.user_scn.ngeom = 1
        execution_model.viewer.sync()

        accept = input("Accept? (y/n)")
        if accept == "y":
            break

    print("goal_obstacle_pos: ", goal_obstacle_pos)
    print("goal_obstacle_quat: ", goal_obstacle_quat)

    def quaternion_distance_symmetric(q1, q2, symmetry_rotations=2):    
        """
        Calculate the minimum distance between quaternions considering symmetry.
        Args:
            q1, q2: quaternions to compare
            symmetry_rotations: number of symmetric rotations (e.g., 4 for 90-degree rotations)
        Returns:
            Minimum distance considering symmetry (between 0 and 1)
        """
        min_dist = float('inf')
        # Create rotation around vertical axis
        for i in range(symmetry_rotations):
            angle = i * (2 * np.pi / symmetry_rotations)
            # Create quaternion for symmetric rotation
            sym_rot = R.from_euler('z', angle).as_quat()
            # Apply symmetric rotation to q2
            q2_sym = R.from_quat(q2).as_matrix() @ R.from_quat(sym_rot).as_matrix()
            q2_sym = R.from_matrix(q2_sym).as_quat()
            # Calculate distance
            dot_product = abs(np.dot(q1, q2_sym))
            dot_product = min(1.0, max(-1.0, dot_product))
            min_dist = min(min_dist, 1 - dot_product)
        return min_dist

    # Update the distance function
    distance_func = lambda x, y: (
        np.linalg.norm(x[0][:2] - y[0][:2]) + 
        0.5 * quaternion_distance_symmetric(x[1], y[1], symmetry_rotations=2)
    )

    # Update the goal check
    goal_check = lambda x: (
        np.linalg.norm(x[0][:2] - goal_obstacle_pos[:2]) < 0.05 and
        quaternion_distance_symmetric(x[1], goal_obstacle_quat, symmetry_rotations=2) < 0.05
    )
    node_id = 0
    current_node = Node((obstacle_pos, obstacle_quat, (qpos, qvel), edge_point_idx, control_input[:2], control_steps), parent=None, node_id=node_id)
    node_id += 1
    current_node.cost_to_come = 0
    current_node.cost_to_go = distance_func((obstacle_pos, obstacle_quat), (goal_obstacle_pos, goal_obstacle_quat))

    queue = []
    heapq.heappush(queue, (current_node.cost_to_go, current_node))
    nodes_dict = {}
    parent_node = None
    prev_edge_point_idx = edge_point_idx
    
    start_time = time.time()
    termination_time = 120
    termination_cond = False
    random_controls = False
    while execution_model.viewer.is_running() and len(queue) > 0:
        # priority based selection
        _, node = heapq.heappop(queue)
        current_obstacle_pos, current_obstacle_quat, current_generalized_state, prev_edge_point_idx, _, control_steps = node.state

        # check if goal is reached
        if goal_check((current_obstacle_pos, current_obstacle_quat)):
            print("Goal reached")
            break
        
        if time.time() - start_time > termination_time:
            termination_cond = True
            break
        
        current_qpos, current_qvel = current_generalized_state

        num_samples = 12
        # 50 random expansions
        for sample_idx in range(num_samples):

            edge_point_idx = sample_idx # np.random.randint(0, len(execution_model.edge_points['obstacle_1_movable'].edge_points))
            
            # reset to current state
            execution_model.data.qpos[:] = current_qpos
            execution_model.data.qvel[:] = 0.0
            control_input[:] = 0.0
            execution_model.step(control_input, 1)

            obstacle_state = execution_model.get_obstacle_states()
            obstacle_pos = obstacle_state['obstacle_1_movable']['position'].copy()
            obstacle_quat = R.from_euler('xyz', obstacle_state['obstacle_1_movable']['orientation'].copy()).as_quat()

            edge_point = execution_model.edge_points['obstacle_1_movable'].edge_points[edge_point_idx]
            execution_model.data.qpos[:2] = edge_point - execution_model.robot_rel_pos[:2]
            execution_model.step(control_input, 1)


            collisions = ["robot"] + execution_model.static_obstacle_names

            continue_flag = False
            for i in range(execution_model.data.ncon):
                
                body1_name = execution_model.model.geom(execution_model.data.contact[i].geom1).name
                body2_name = execution_model.model.geom(execution_model.data.contact[i].geom2).name
                
                if body1_name in collisions and body2_name in collisions:
                    continue_flag = True
                    break

                if (body1_name == "robot" and "movable" in body2_name) or ("movable" in body1_name and body2_name == "robot"):
                    continue_flag = True
                    if (body1_name == "robot" and body2_name == "obstacle_1_movable") or (body1_name == "obstacle_1_movable" and body2_name == "robot"):
                        continue_flag = False
                    break

            if continue_flag:
                continue

            robot_state = execution_model.get_robot_state()
            robot_pos = robot_state['position']
            com = execution_model.edge_points['obstacle_1_movable'].get_mid_point(edge_point_idx)
            angle = np.arctan2(com[1] - robot_pos[1], com[0] - robot_pos[0])

            obstacle_pos = execution_model.obstacle_states['obstacle_1_movable']['position'].copy()

            scaling = 0.3
            control_steps = 500
            if distance_func((obstacle_pos, obstacle_quat), (goal_obstacle_pos, goal_obstacle_quat)) > 0.5:
                scaling = 0.5
                control_steps = 1000
            # control randomness
            # scaling = 0.3
            # if random_controls:
            #     scaling = np.random.uniform(0, 1.0)

            control_input[:2] = scaling * np.array([np.cos(angle), np.sin(angle)]) # randomness 1
            # if random_controls:
            #     control_steps = np.random.randint(100, 1000) # randomness 2
            # else:
            #     control_steps = 500
            
            execution_model.step(control_input, control_steps, viewer=True)
            new_generalized_state = (execution_model.get_qpos(), execution_model.get_qvel())
            new_obstacle_pos = execution_model.obstacle_states['obstacle_1_movable']['position'].copy()
            new_obstacle_quat = R.from_euler('xyz', obstacle_state['obstacle_1_movable']['orientation'].copy()).as_quat()
            child_node = Node((new_obstacle_pos, new_obstacle_quat, new_generalized_state, edge_point_idx, control_input[:2].copy(), control_steps), parent=node, node_id=node_id)

            node_id += 1

            child_node.cost_to_come = node.cost_to_come + distance_func((current_obstacle_pos, current_obstacle_quat), (new_obstacle_pos, new_obstacle_quat))
            child_node.cost_to_go = distance_func((new_obstacle_pos, new_obstacle_quat), (goal_obstacle_pos, goal_obstacle_quat))

            continue_flag = True
            for i in range(execution_model.data.ncon):
                body1_name = execution_model.model.geom(execution_model.data.contact[i].geom1).name
                body2_name = execution_model.model.geom(execution_model.data.contact[i].geom2).name

                if ("robot" in body1_name and "obstacle_1_movable" in body2_name) or ("obstacle_1_movable" in body1_name and "robot" in body2_name):
                    continue_flag = False
                    break
                    # break
            if continue_flag:
                continue

            extra_cost = 0.0
            if sample_idx != prev_edge_point_idx:
                extra_cost = 0.01
            
            heapq.heappush(queue, (child_node.cost_to_go + extra_cost, child_node))
            
            
    if termination_cond:
        print("Termination condition met")
    else:
        time.sleep(2)
        final_idxs = []
        final_control_inputs = []
        final_control_steps = []
        while node != None:
            final_idxs.append(node.state[3])
            final_control_inputs.append(node.state[4])
            final_control_steps.append(node.state[5])
            node = node.parent

        final_edge_point_idxs = final_idxs[::-1]
        final_control_inputs = final_control_inputs[::-1]
        final_control_steps = final_control_steps[::-1]

        for _ in range(2):

            # resetting to initial state
            mujoco.mj_resetData(execution_model.model, execution_model.data)
            control_input[:] = 0.0
            execution_model.step(control_input, 1)
        
            execution_model.viewer.user_scn.ngeom = 0
            mujoco.mjv_initGeom(execution_model.viewer.user_scn.geoms[0], type=mujoco.mjtGeom.mjGEOM_BOX, size=execution_model.movable_obstacle_sizes[0], pos=[goal_obstacle_pos[0], goal_obstacle_pos[1], goal_obstacle_pos[2]/2], mat=R.from_quat(goal_obstacle_quat).as_matrix().flatten(), rgba=[0, 1, 0, 0.25])
            execution_model.viewer.user_scn.ngeom = 1
            execution_model.viewer.sync()

            for i, edge_point_idx in enumerate(final_edge_point_idxs):
                edge_point = execution_model.edge_points['obstacle_1_movable'].edge_points[edge_point_idx]
                execution_model.data.qpos[:2] = edge_point - execution_model.robot_rel_pos[:2]
                execution_model.data.qvel[:] = 0.0
                execution_model.step(control_input, 1, viewer=True)

                robot_state = execution_model.get_robot_state()
                robot_pos = robot_state['position']
                com = execution_model.edge_points['obstacle_1_movable'].get_mid_point(edge_point_idx)
                angle = np.arctan2(com[1] - robot_pos[1], com[0] - robot_pos[0])     
                control_input[:2] = final_control_inputs[i]
                execution_model.step(control_input, final_control_steps[i], viewer=True)
                execution_model.viewer.sync()
                time.sleep(0.1)
            time.sleep(1)

    execution_model.viewer.close()

if __name__ == "__main__":

    main()
