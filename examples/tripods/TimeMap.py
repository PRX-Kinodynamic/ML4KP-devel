import sys
import os
import math
import random
# Remember to add libpyDirtMP to your PYTHONPATH
# On bash: ``export PYTHONPATH=$DIRTMP_PATH/lib/:$PYTHONPATH
import libpyDirtMP as prx
import numpy as np
import torch


class TimeMap:

    def __init__(self, system_type, time_step, parameters="examples/tripods/ackermann_ha_roa.yaml"):
        """Create a time map for a given example (system_type);
        time_step * simulation_step = time in seconds;
        parameters = load the parameters of the system_type
        """

        self.time_step = time_step
        params = prx.param_loader(parameters, sys.argv)

        self.checker_type = params["checker_type"].as_string()
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

        self.ss.copy_point_from_vector(
            self.start_state, params["/plant/start_state"].as_float_vector())
        self.ss.copy_point_from_vector(
            self.goal_state, params["/plant/goal_state"].as_float_vector())
        self.ss.copy_from_point(self.start_state)

        ss_dim = self.ss.get_dimension()
        cs_dim = self.cs.get_dimension()

        self.ss.print_bounds()
        self.cs.print_bounds()

        if system_type == "ackermann_hyb":
            self.ctrl_1 = prx.ackermann_FO_ctrl(self.plant, "ackermann_FO_ctrl_1")
            self.ctrl_2 = prx.ackermann_FO_ctrl(self.plant, "ackermann_FO_ctrl_2")

            k_rho_1 = +1.0
            k_alpha_1 = +9.5
            k_beta_1 = -9.0

            k_rho_2 = +1.0
            k_alpha_2 = +9.5
            k_beta_2 = -9.5

            self.ctrl_1.set_gains(k_rho_1, k_alpha_1, k_beta_1)
            self.ctrl_2.set_gains(k_rho_2, k_alpha_2, k_beta_2)

            self.ctrl_1.set_goal(self.goal_state)
            self.ctrl_2.set_goal(self.goal_state)

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
            self.cs.copy_from_point(ctrl_pt)
            self.cs.enforce_bounds()
            self.plant.propagate(self.simulation_step)
            self.ss.copy_to_point(self.start_state)

            duration_so_far += self.simulation_step

        self.ss.copy_to_point(self.end_state)
        return [self.end_state[0], self.end_state[1]]

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
        self.ss.copy_from_vector(X)
        checker = prx.condition_check(
            self.checker_type, self.time_step)

        while True:

            if (-1.9 <= self.end_state[0] and
                    -1.8 <= self.end_state[1] and self.end_state[1] <= 1.2):
                self.ctrl_2.compute_controls()
            else:
                self.ctrl_1.compute_controls()
            self.cs.enforce_bounds()

            self.plant.propagate(self.simulation_step)
            self.ss.copy_to_point(self.end_state)

            if checker.check():
                break

        self.ss.copy_to_point(self.end_state)
        return [self.end_state[0], self.end_state[1], self.end_state[2]]
