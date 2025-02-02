import mujoco
import mujoco.viewer
import numpy as np
import cvxpy as cp
import time
from scipy.optimize import minimize

def add_line(viewer, start, end, color):
    # Calculate the midpoint and direction
    midpoint = (start + end) / 2
    direction = end - start
    length = np.linalg.norm(direction)
    if length == 0:
        return  # Avoid division by zero
    direction /= length  # Normalize direction

    # Define the size and orientation of the line
    size = [0.005, 0.005, length / 2]  # Adjust thickness as needed

    # Define a 3x3 identity matrix
    rotation_matrix = np.eye(3)

    # Flatten the matrix to a 1D array
    flattened_matrix = rotation_matrix.flatten()

    # Initialize an array to hold the quaternion
    quat = np.zeros(4)

    # Perform the conversion
    mujoco.mju_mat2Quat(quat, flattened_matrix)

    # Add the line as a cylinder
    viewer.add_geom(
        pos=midpoint,
        size=size,
        rgba=color,
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,
        mat=quat
    )

class MPCController:
    def __init__(self, model, dt, horizon=20):
        self.model = model
        self.dt = dt
        self.horizon = horizon
        
        # MPC parameters
        self.Q = np.diag([5.0, 5.0])      # Object position error cost
        self.R = np.diag([1.0, 1.0])      # Control input cost
        
        # Find relevant body IDs
        self.robot_id = model.body('ball').id
        self.object_id = model.body('rigid_body').id
        
        # Contact parameters
        self.desired_contact_dist = 0.225  # Desired distance between centers
        self.contact_weight = 100.0       # Weight for contact maintenance
        
        # Visualization parameters
        self.traces = []  # Store trajectory traces
        self.elite_traces = []  # Store elite trajectory traces
        self.trace_duration = 1.0  # How long to show traces (seconds)
        self.last_trace_time = 0
        
        # Add action smoothing parameters
        self.S = np.diag([1.0, 1.0])  # Action smoothing cost matrix
        self.prev_action = np.zeros(2)  # Store previous action
        self.prev_optimal_trajectory = np.zeros((horizon, 2))
        
        # MPPI parameters
        self.mppi_samples = 50
        self.mppi_temperature = 0.1
        self.mppi_noise_std = 1.0
        
        # Control constraints
        self.control_limits = np.array([
            [-5.0, 5.0],  # x control limits
            [-5.0, 5.0]   # y control limits
        ])
        
        # State-dependent temperature parameters
        self.base_temperature = 0.1
        self.max_temperature = 1.0
        self.distance_scale = 2.0  # Scale factor for distance-based temperature
        
    def simulate_step(self, cp, cv, control):
        temp_data = mujoco.MjData(self.model)
        
        # Set the state
        temp_data.qpos = cp  # position
        temp_data.qvel = cv  # velocity
        
        # Set the control
        temp_data.ctrl[self.model.actuator('actuator_x').id] = control[0]
        temp_data.ctrl[self.model.actuator('actuator_y').id] = control[1]
        
        # Step the simulation
        mujoco.mj_step(self.model, temp_data)
        
        # next_state = np.concatenate([
        #     temp_data.qpos[:2],
        #     temp_data.qvel[:2]
        # ])
        
        # Get positions and velocities
        robot_pos = temp_data.xpos[self.robot_id][:2]
        object_pos = temp_data.xpos[self.object_id][:2]
        robot_vel = temp_data.qvel[0:2]  # Assuming first 2 DOFs are robot
        object_vel = temp_data.qvel[2:4]  # Assuming next 2 DOFs are object
        
        return temp_data.qpos.copy(), temp_data.qvel.copy(), robot_pos, object_pos, robot_vel, object_vel
        
    def evaluate_trajectory(self, controls, current_position, current_velocity, target_position):
        cost = 0
        cp, cv = current_position.copy(), current_velocity.copy()
        trajectory = []
        
        # Add action smoothing cost for first control
        # action_diff = controls[0] - self.prev_action
        # cost += float(action_diff.T @ self.S @ action_diff)
        
        for t in range(len(controls)):
            # Simulate next state
            cp, cv, robot_pos, object_pos, robot_vel, object_vel = self.simulate_step(cp, cv, controls[t])
            trajectory.append(robot_pos)
            # 1. Object position error cost
            obj_error = object_pos - target_position
            cost += float(obj_error.T @ self.Q @ obj_error)
            
            # 2. Contact maintenance cost
            robot_to_obj = robot_pos - object_pos
            dist_error = np.linalg.norm(robot_to_obj) - self.desired_contact_dist
            cost += self.contact_weight * dist_error
            
            
            # 3. Relative velocity alignment cost
            # Encourage robot velocity to align with object-to-target direction
            rel_vel = robot_vel - object_vel
            rel_vel_norm = np.linalg.norm(rel_vel) + 1e-6
            
            # Direction from object to target
            to_target = target_position - object_pos
            to_target_norm = np.linalg.norm(to_target) + 1e-6
            
            # Penalize misalignment between relative velocity and desired direction
            # alignment = np.dot(rel_vel/rel_vel_norm, to_target/to_target_norm)
            # cost += 2.0 * (1.0 - alignment)
            
            # 4. Control cost
            cost += float(controls[t].T @ self.R @ controls[t])
            
            # Add action smoothing cost between consecutive controls
            # if t < len(controls) - 1:
            #     action_diff = controls[t + 1] - controls[t]
            #     cost += float(action_diff.T @ self.S @ action_diff)

            
        return cost, trajectory
    
    def simulate_trajectory(self, controls, current_position, current_velocity):
        """Simulate full trajectory and return all states"""
        states = []
        robot_positions = []
        object_positions = []

        current_positions = []
        current_velocities = []

        current_position = current_position.copy()
        current_velocity = current_velocity.copy()
        
        # state = current_state.copy()
        
        for control in controls:
            cp, cv, robot_pos, object_pos, _, _ = self.simulate_step(current_position, current_velocity, control)
            # states.append(next_state)
            robot_positions.append(robot_pos)
            object_positions.append(object_pos)
            current_positions.append(cp)    
            current_velocities.append(cv)
            
        return np.array(current_positions), np.array(current_velocities), np.array(robot_positions), np.array(object_positions)
    
    def project_controls(self, controls):
        """Project controls onto feasible set defined by control limits"""
        return np.clip(
            controls,
            self.control_limits[:, 0],
            self.control_limits[:, 1]
        )
    
    def compute_state_dependent_temperature(self, object_pos, target_position):
        """Compute temperature based on distance to target"""
        distance = np.linalg.norm(object_pos - target_position)
        # Temperature increases with distance from target
        temperature = self.base_temperature + \
                     (self.max_temperature - self.base_temperature) * \
                     np.tanh(distance / self.distance_scale)
        return temperature
    
    def optimize_mppi(self, current_position, current_velocity, target_position, sim_time):
        # Initialize controls as before
        nominal_controls = np.zeros((self.horizon, 2))
        if hasattr(self, 'prev_optimal_trajectory'):
            nominal_controls[:-1] = self.prev_optimal_trajectory[1:]
            nominal_controls[-1] = self.prev_optimal_trajectory[-1]
        
        # Get current object position for state-dependent temperature
        _, _, _, object_pos, _, _ = self.simulate_step(
            current_position, 
            current_velocity, 
            nominal_controls[0]
        )
        
        # Compute state-dependent temperature
        temperature = self.compute_state_dependent_temperature(object_pos, target_position)
        
        # Generate control noise with built-in cost
        noise = np.random.normal(0, self.mppi_noise_std, (self.mppi_samples, self.horizon, 2))
        
        # Add control cost directly in noise sampling
        control_cost = 0.1  # Adjust this weight
        noise_cost = control_cost * np.sum(noise**2, axis=(1, 2))
        
        # Evaluate trajectories (can be parallelized)
        trajectories = []
        costs = np.zeros(self.mppi_samples)
        
        # Parallel evaluation using joblib
        from joblib import Parallel, delayed
        
        def evaluate_sample(noise_sample):
            perturbed_controls = self.project_controls(nominal_controls + noise_sample)
            cost, trajectory = self.evaluate_trajectory(
                perturbed_controls,
                current_position,
                current_velocity,
                target_position
            )
            return cost, trajectory
        
        # Parallel execution
        results = Parallel(n_jobs=-1)(
            delayed(evaluate_sample)(noise[i]) 
            for i in range(self.mppi_samples)
        )
        
        # Unpack results
        for i, (cost, trajectory) in enumerate(results):
            costs[i] = cost + noise_cost[i]  # Add noise cost
            trajectories.append(trajectory)
        
        # Compute weights with state-dependent temperature
        beta = 1.0 / temperature
        min_cost = np.min(costs)
        weights = np.exp(-beta * (costs - min_cost))
        weights = weights / (np.sum(weights) + 1e-10)
        
        # Update nominal controls
        weighted_noise = np.sum(weights[:, None, None] * noise, axis=0)
        optimal_controls = self.project_controls(nominal_controls + weighted_noise)
        
        # Store for next iteration
        self.prev_optimal_trajectory = optimal_controls
        self.prev_action = optimal_controls[0]
        
        # Get elite trajectories
        K = 3
        elite_indices = np.argsort(weights)[-K:]
        elite_trajectories = [trajectories[i] for i in elite_indices]
        
        return optimal_controls[0], trajectories, elite_trajectories
    
    def render_traces(self, viewer, trajectories, elite_trajectories):
        """Render sampled and elite trajectories"""
        viewer.user_scn.ngeom = 0
        ngeom = 0

        # Render random samples (gray)
        # for traj in trajectories[:10]:  # Show first 10 trajectories
        #     for pos in traj:
        #         print(pos)
        #         if ngeom >= 100:
        #             break
        #         mujoco.mjv_initGeom(
        #             viewer.user_scn.geoms[ngeom],
        #             type=mujoco.mjtGeom.mjGEOM_SPHERE,
        #             size=[0.01, 0.01, 0.01],
        #             pos=[pos[0], pos[1], 0.2],
        #             mat=np.eye(3).flatten(),
        #             rgba=[0., 1.0, 0.0, 0.3]
        #         )
        #         ngeom += 1
        # viewer.user_scn.ngeom = ngeom
        # viewer.sync()
        
        # Render elite samples (blue)
        for traj in elite_trajectories[:1]:  # Show top 3 elite trajectories
            for pos in traj:
                if ngeom >= 200:
                    break
                mujoco.mjv_initGeom(
                    viewer.user_scn.geoms[ngeom],
                    type=mujoco.mjtGeom.mjGEOM_SPHERE,
                    size=[0.01, 0.01, 0.01],
                    pos=[pos[0], pos[1], 0.2],
                    mat=np.eye(3).flatten(),
                    rgba=[1, 1, 1, 0.5]
                )
                ngeom += 1
        
        viewer.user_scn.ngeom = ngeom
        viewer.sync()

# Load the MuJoCo model
model = mujoco.MjModel.from_xml_path('resources/models/welded_push/model.xml')
data = mujoco.MjData(model)

# Initialize the simulation
mujoco.mj_resetData(model, data)



desired_x = 0.5
desired_y = 0.25
tolerance = 0.1
target_position = np.array([desired_x, desired_y])

# Initialize MPC controller
mpc = MPCController(
    model=model,
    dt=model.opt.timestep,
    horizon=100  
)

# Create the viewer
viewer = mujoco.viewer.launch_passive(model, data)

# Simulation loop
sim_time = 0.0
while viewer.is_running():
    # Get current state
    # robot qpos

    current_position = data.qpos
    current_velocity = data.qvel
    
    # Compute control inputs using MPPI
    control_inputs, all_trajectories, elite_trajectories = mpc.optimize_mppi(
        current_position, current_velocity, target_position, sim_time
    )
    
    # Apply control inputs
    data.ctrl[model.actuator('actuator_x').id] = control_inputs[0]
    data.ctrl[model.actuator('actuator_y').id] = control_inputs[1]
    
    # Step the simulation
    mujoco.mj_step(model, data)
    # Render traces
    mpc.render_traces(viewer, all_trajectories, elite_trajectories)
    sim_time += model.opt.timestep
    
    # Render the simulation
    viewer.sync()

    rigid_body_position = data.xpos[model.body('rigid_body').id][:2]
    
    # Check termination
    if np.linalg.norm(target_position - rigid_body_position) < tolerance:
        break

viewer.close()