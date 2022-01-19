import sys
import os
import math
import random
# Remember to add libpyDirtMP to your PYTHONPATH
# On bash: ``export PYTHONPATH=$DIRTMP_PATH/lib/:$PYTHONPATH
import torch
import libpyDirtMP as prx
import numpy as np
from scipy.spatial.transform import Rotation as R


class TimeMap:

    def __init__(self, system_type, time_step, parameters="examples/tripods/ackermann_ha_roa.yaml"):
        """Create a time map for a given example (system_type);
        time_step * simulation_step = time in seconds;
        parameters = load the parameters of the system_type
        """
        if isinstance(parameters, str):
            params = prx.param_loader(parameters, sys.argv)
        elif isinstance(parameters, prx.param_loader):
            print("Instance of prx.param_loader")
            params = parameters

        self.time_step = time_step
        self.params = params
        self.Q = None
        self.R = None
        self.K = None
        self.lqr = None

        self.ctrl_1 = None
        self.ctrl = None
        self.checker_type = params["checker_type"].as_string()
        self.checker_value = params["checker_value"].as_float()
        self.simulation_step = params["simulation_step"].as_float()
        prx.set_simulation_step(self.simulation_step)
        prx.init_random(params["random_seed"].as_int())

        obstacles = prx.load_obstacles(params["environment"].as_string())
        obstacle_list = obstacles.objects
        obstacle_names = obstacles.names

        plant_name = params["/plant/name"].as_string()
        plant_path = params["/plant/path"].as_string()
        self.plant = prx.system_factory.create_system(plant_name, plant_path)
        if self.plant == None:
            print("Error: plant not found!")
            exit(-1)

        wm = prx.world_model([self.plant], obstacle_list)
        wm.create_context("context", [plant_name], obstacle_names)
        context = wm.get_context("context")

        self.ss = context.system_group.get_state_space()
        self.cs = context.system_group.get_control_space()
        self.ps = self.plant.get_parameter_space()

        lower_bounds = params["/plant/state_space_lower_bound"].as_float_vector()
        upper_bounds = params["/plant/state_space_upper_bound"].as_float_vector()
        self.ss.set_bounds(lower_bounds, upper_bounds)

        cs_lb = params["/plant/control_space_lower_bound"].as_float_vector()
        cs_up = params["/plant/control_space_upper_bound"].as_float_vector()
        self.cs.set_bounds(cs_lb, cs_up)

        self.start_state = self.ss.make_point()
        self.goal_state = self.ss.make_point()
        self.end_state = self.ss.make_point()
        self.ctrl_pt = self.cs.make_point()

        self.u_goal = self.cs.make_point()
        for i in range(len(self.u_goal)): self.u_goal[i] = 0

        self.ss.copy_point_from_vector(
            self.start_state, params["/plant/start_state"].as_float_vector())
        self.ss.copy_point_from_vector(
            self.goal_state, params["/plant/goal_state"].as_float_vector())
        self.ss.copy_from_point(self.start_state)

        ss_dim = self.ss.get_dimension()
        cs_dim = self.cs.get_dimension()

        self.ss.print_bounds()
        self.cs.print_bounds()

        self.radius = params["goal_region_radius"].as_float()

        # if system_type == "ackermann_hyb":
            # self.ctrl_1 = prx.ackermann_FO_ctrl(self.plant, "ackermann_FO_ctrl_1")
            # self.ctrl_2 = prx.ackermann_FO_ctrl(self.plant, "ackermann_FO_ctrl_2")

            # k_rho_1 = +1.0
            # k_alpha_1 = +9.5
            # k_beta_1 = -9.0

            # k_rho_2 = +1.0
            # k_alpha_2 = +9.5
            # k_beta_2 = -9.5

            # self.ctrl_1.set_gains(k_rho_1, k_alpha_1, k_beta_1)
            # self.ctrl_2.set_gains(k_rho_2, k_alpha_2, k_beta_2)

            # self.ctrl_1.set_goal(self.goal_state)
            # self.ctrl_2.set_goal(self.goal_state)

        if system_type == 'ackermann_lqr':
            u_goal = self.cs.make_point()

            u_goal[0] = 0
            u_goal[1] = 1

            self.plant.linearize(goal_state, u_goal)

            Q = prx.matrix.Identity(ss_dim, ss_dim)
            R = prx.matrix.Identity(cs_dim, cs_dim)
            q_vec = params["/plant/lqr_Q"].as_float_vector()

            for i in range(ss_dim):
                Q[i, i] = q_vec[i]

            self.lqr = prx.lqr(self.plant, Q, R, "LQR")
            self.lqr.set_goal(goal_state)
            self.lqr.compute_K()

        if system_type == "ackermann_lc":
            controller_path = params["controller_path"].as_string()
            controller_path = prx.lib_path + controller_path
            self.controller = torch.load(controller_path)
            self.controller.eval()
            torch.manual_seed(params["random_seed"].as_int())
            self.radius = params["goal_region_radius"].as_float()

        if system_type == "pendulum_lc":
            controller_path = params["controller_path"].as_string()
            controller_path = prx.lib_path + controller_path
            self.controller = torch.load(controller_path)
            self.controller.eval()
            torch.manual_seed(params["random_seed"].as_int())
            self.radius = params["goal_region_radius"].as_float()
            
            

        params.print()

    def pendulum_lc(self, X):
        self.ss.copy_from_vector(X)
        self.ss.copy_to_point(self.start_state)

        ctrl_input = torch.zeros(1, 4)

        duration_so_far = 0
        ctrl = [0]
        while duration_so_far <= self.time_step and prx.space_t.euclidean_2d(self.start_state, self.goal_state, 0, 2) > self.radius:
            ctrl_input[0, 0] = self.start_state[0]
            ctrl_input[0, 1] = self.start_state[1]
            ctrl_input[0, 2] = self.goal_state[0]
            ctrl_input[0, 3] = self.goal_state[1]

            with torch.no_grad():
                ctrl_output = self.controller(ctrl_input)[0].cpu()

            # ctrl = [-0.6371781908344007 + ((ctrl_output + 1.)*0.6371781908344007)]
            ctrl = np.array([-0.6371781908344007 + ((ctrl_output + 1.)
                                                    * 0.6371781908344007)], dtype=np.float64)

            self.ctrl_pt[0] = ctrl[0]
            self.cs.copy_from_point(self.ctrl_pt)
            self.cs.enforce_bounds()
            self.plant.propagate(self.simulation_step)
            self.ss.copy_to_point(self.start_state)

            duration_so_far += self.simulation_step

        self.ss.copy_to_point(self.end_state)
        return [self.end_state[0], self.end_state[1]]

    def pendulum_lqr(self, X):
        self.ss.copy_from_vector(X)
        self.ss.copy_to_point(self.start_state)

        if self.lqr == None:
            self.plant.linearize(self.goal_state, self.u_goal)
            self.Q = prx.matrix.Identity(2, 2)
            self.R = prx.matrix.Identity(1, 1)
            self.lqr = prx.lqr(self.plant, self.Q, self.R, "LQR")
            self.lqr.set_goal(self.goal_state)
            self.lqr.compute_K()
            self.K = self.lqr.get_K()
            self.ps[1] = self.params["/plant/friction"].as_float()

        duration_so_far = 0
        while duration_so_far <= self.time_step and prx.space_t.euclidean_2d(self.start_state, self.goal_state, 0, 2) > self.radius:
            self.lqr.compute_controls()
            self.cs.enforce_bounds()
            self.plant.propagate(self.simulation_step)
            self.ss.copy_to_point(self.start_state)

            duration_so_far += self.simulation_step

        self.ss.copy_to_point(self.end_state)
        return [self.end_state[0], self.end_state[1]]

    def acrobot_lqr(self, X):
        self.ss.copy_from_vector(X)
        self.ss.copy_to_point(self.start_state)

        if self.lqr == None:
            self.plant.linearize()
            self.Q = prx.matrix.Identity(4, 4)
            self.Q[0, 0] = 10
            self.Q[1, 1] = 10
            self.Q[2, 2] = 1
            self.Q[3, 3] = 1
            self.R = prx.matrix.Identity(1, 1)
            self.lqr = prx.lqr(self.plant, self.Q, self.R, "LQR")
            self.lqr.set_goal(self.goal_state)
            self.lqr.compute_K()
            self.K = self.lqr.get_K()
            self.radius = self.params["goal_region_radius"].as_float()
            self.ps[0] = self.params["/plant/mass"].as_float()
            self.ps[1] = self.params["/plant/g"].as_float()
            # self.ps[1] = self.params["/plant/friction"].as_float()

        duration_so_far = 0
        while duration_so_far <= self.time_step and prx.space_t.euclidean_2d(self.start_state, self.goal_state, 0, 4) > self.radius:
            self.lqr.compute_controls()
            self.cs.enforce_bounds()
            self.plant.propagate(self.simulation_step)
            self.ss.copy_to_point(self.start_state)

            duration_so_far += self.simulation_step

        self.ss.copy_to_point(self.end_state)
        return self.end_state.to_list() # [self.end_state[0], self.end_state[1]]

    def pendulum_no_ctrl(self, X):
        self.ss.copy_from_vector(X)
        self.ss.copy_to_point(self.start_state)

        # if self.lqr == None:
        #     self.plant.linearize()
        #     self.Q = prx.matrix.Identity(2,2)
        #     self.R = prx.matrix.Identity(1,1)
        #     self.lqr = prx.lqr(self.plant, self.Q, self.R, "LQR");
        #     self.lqr.compute_K();
        #     self.K = self.lqr.get_K();

        duration_so_far = 0
        while duration_so_far <= self.time_step and prx.space_t.euclidean_2d(self.start_state, self.goal_state, 0, 2) > self.radius:
            # self.lqr.compute_controls();
            self.cs.enforce_bounds()
            self.plant.propagate(self.simulation_step)
            self.ss.copy_to_point(self.start_state)

            duration_so_far += self.simulation_step

        self.ss.copy_to_point(self.end_state)
        return [self.end_state[0], self.end_state[1]]

    def acrobot_no_ctrl(self, X):
        self.ss.copy_from_vector(X)
        self.ss.copy_to_point(self.start_state)

        duration_so_far = 0
        while duration_so_far <= self.time_step :
            self.cs.enforce_bounds()
            self.plant.propagate(self.simulation_step)
            self.ss.copy_to_point(self.start_state)

            duration_so_far += self.simulation_step

        self.ss.copy_to_point(self.end_state)
        return self.end_state.to_list() # [self.end_state[0], self.end_state[1]]

    def ackermann_lc(self, X):
        self.ss.copy_from_vector(X)
        self.ss.copy_to_point(self.start_state)
        # solution_traj = prx.trajectory(self.ss)

        ctrl_input = torch.zeros(1, 6)

        duration_so_far = 0

        while duration_so_far <= self.time_step and prx.space_t.euclidean_2d(self.start_state, self.goal_state, 0, 3) > self.radius:
            ctrl_input[0, 0] = self.start_state[0]
            ctrl_input[0, 1] = self.start_state[1]
            ctrl_input[0, 2] = self.start_state[2]
            ctrl_input[0, 3] = self.goal_state[0]
            ctrl_input[0, 4] = self.goal_state[1]
            ctrl_input[0, 5] = self.goal_state[2]

            with torch.no_grad():
                ctrl_output = self.controller(ctrl_input)[0].cpu()
            # ctrl = np.array(
            #     [np.array([-np.pi/3 + ((ctrl_output[0] + 1)*np.pi/3), (ctrl_output[1]+1)*15], dtype=np.float64)])

            ctrl = [-np.pi/3 + ((ctrl_output[0].item() + 1)*np.pi/3),
                    (ctrl_output[1].item() + 1)*15]
            self.cs.copy_from_vector(ctrl)
            self.cs.enforce_bounds()
            self.plant.propagate(0.1)
            self.ss.copy_to_point(self.start_state)
            # solution_traj.copy_onto_back(self.start_state)

            duration_so_far += 0.1

        self.ss.copy_to_point(self.end_state)
        return [self.end_state[0], self.end_state[1], self.end_state[2]]

    def ackermann_lqr(self, X):
        self.ss.copy_from_vector(X)

        checker = prx.condition_check(
            self.checker_type, self.time_step)

        while True:
            self.lqr.compute_controls()

            self.cs.enforce_bounds()
            self.plant.propagate(self.simulation_step)

            self.ss.copy_to_point(self.end_state)

            if checker.check():
                break

        self.ss.copy_to_point(self.end_state)
        return [self.end_state[0], self.end_state[1], self.end_state[2]]

    def ackermann_hyb(self, X):
        if self.ctrl == None:
            self.ctrl = prx.ackermann_FO_ctrl(self.plant, "ackermann_FO_ctrl_1")

            k_rho_1     = +1.0
            k_alpha_1   = +5.0
            k_beta_1    = -2.0

            self.ctrl.set_gains(k_rho_1, k_alpha_1, k_beta_1)
            # self.ctrl_2.set_gains(k_rho_2, k_alpha_2, k_beta_2)

        self.ctrl.set_goal(self.goal_state)
        # self.ctrl_2.set_goal(self.goal_state)

        self.ss.copy_from_vector(X)

        checker = prx.condition_check(self.checker_type, self.checker_value)

        # r = R.from_euler('z', -np.pi/2.0 - self.goal_state[2], degrees=False)
        # v1 = [-1.9, -1.8, -1.57]
        # v2 = [ 0  ,  1.2, -1.57]
    
        # v1 = r.apply(v1)
        # v2 = r.apply(v2)

        while True:

            # if (-1.9 <= self.end_state[0] and
                    # -1.8 <= self.end_state[1] and self.end_state[1] <= 1.2):
            # if (v1[0] <= X[0] and
            #     v1[1] <= X[1] <= v2[1]):
                # self.ctrl_2.compute_controls()
            # else:
                # self.ctrl_1.compute_controls()
            self.ctrl.compute_controls()
            self.cs.enforce_bounds()

            self.plant.propagate(self.simulation_step)
            self.ss.copy_to_point(self.end_state)

            if checker.check():
                break

        # self.ss.copy_to_point(self.end_state)
        return self.end_state.to_list()

if __name__ == "__main__":
    # Adding this for convenient testing... 
    # TODO: Check if there is a better way of doing this
    

    time = 10
    yaml_file = "examples/tripods/lqr_roa.yaml"
    params = prx.param_loader(yaml_file)
    # params = prx.param_loader(yaml_file, sys.argv)

    params["/plant/friction"] = 0.05
    params["/plant/g"] = 9.81 # Earth
    # params["/plant/g"] = 3.721 # Mars
    # params["/plant/g"] = 24.79 # Jupiter
    params["/plant/mass"] = 1 # Normal
    # params["/plant/mass"] = 10 # Heavy
    # TM = TimeMap("acrobot_lqr", time, yaml_file)
    TM = TimeMap("acrobot_lqr", time, params)
                    #  "examples/tripods/lc_roa.yaml")
                    #  "examples/tripods/lqr.yaml")

    # start_state_vector = [3.14, 0.0]
    start_state_vector = [0.0, 0.0, 0, 0]

    def g(X):
        # return TM.pendulum_lc(X)
        # return TM.pendulum_lqr(X)
        return TM.acrobot_lqr(X)
        # return TM.acrobot_no_ctrl(X)




    print( "g:", g(start_state_vector) )
    