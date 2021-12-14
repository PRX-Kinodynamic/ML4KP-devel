import sys
import math
import random
# Remember to add libpyDirtMP to your PYTHONPATH
# On bash: ``export PYTHONPATH=$DIRTMP_PATH/lib/:$PYTHONPATH
import libpyDirtMP as prx
import numpy as np


class TimeMap:

    def __init__(self, system_type, time_step, parameters="examples/tripods/ackermann_ha_roa.yaml"):
        # """Create a time map for a given example (system_type);
        # time_step * simulation_step = time in seconds;
        # parameters = load the parameters of the system_type
        # """

        params = prx.param_loader(parameters, sys.argv)

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

        start_state = self.ss.make_point()
        goal_state = self.ss.make_point()
        self.end_state = self.ss.make_point()

        self.ss.copy_point_from_vector(start_state, params["/plant/start_state"].as_float_vector())
        self.ss.copy_point_from_vector(goal_state, params["/plant/goal_state"].as_float_vector())
        self.ss.copy_from_point(start_state)

        self.checker = prx.condition_check(
            params["checker_type"].as_string(), time_step)

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

            self.ctrl_1.set_goal(goal_state)
            self.ctrl_2.set_goal(goal_state)

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
            print("jas")

        params.print()

    def ackermann_lqr(self, X):
        self.ss.copy_from_vector(X)

        while True:
            self.lqr.compute_controls()

            self.cs.enforce_bounds()
            self.plant.propagate(self.simulation_step)

            self.ss.copy_to_point(self.end_state)

            if self.checker.check():
                break

        self.ss.copy_to_point(self.end_state)
        return [self.end_state[0], self.end_state[1], self.end_state[2]]

    def ackermann_hyb(self, X):
        self.ss.copy_from_vector(X)

        while True:

            if (-1.9 <= self.end_state[0] and
                    -1.8 <= self.end_state[1] and self.end_state[1] <= 1.2):
                self.ctrl_2.compute_controls()
            else:
                self.ctrl_1.compute_controls()
            self.cs.enforce_bounds()

            self.plant.propagate(self.simulation_step)
            self.ss.copy_to_point(self.end_state)

            if self.checker.check():
                break

        self.ss.copy_to_point(self.end_state)
        return [self.end_state[0], self.end_state[1], self.end_state[2]]
