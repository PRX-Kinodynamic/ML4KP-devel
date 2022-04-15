import sys 
import os 
import torch 
import libpyDirtMP as prx 
import numpy as np 

class NoisyTimeMap:
    def __init__(self, system_type, time_step, parameters):
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

        self.simulation_step = params["simulation_step"].as_float()
        prx.set_simulation_step(self.simulation_step)
        prx.init_random(params["random_seed"].as_int())

        plant_name = params["/plant/name"].as_string()
        plant_path = params["/plant/path"].as_string()
        self.plant = prx.system_factory.create_system(plant_name, plant_path)
        if self.plant == None:
            print("Error: plant not found!")
            exit(-1)

        wm = prx.world_model([self.plant], [])
        wm.create_context("context", [plant_name], [])
        self.context = wm.get_context("context")

        self.ss = self.context.system_group.get_state_space()
        self.cs = self.context.system_group.get_control_space()
        self.ctrl = self.cs.make_point()
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

        self.u_goal = self.cs.make_point()
        for i in range(len(self.u_goal)): self.u_goal[i] = 0

        self.ss.copy_point_from_vector(
            self.start_state, params["/plant/start_state"].as_float_vector())
        self.ss.copy_point_from_vector(
            self.goal_state, params["/plant/goal_state"].as_float_vector())
        self.ss.copy_from_point(self.start_state)

        self.ss.print_bounds()
        self.cs.print_bounds()

        self.radius = params["goal_region_radius"].as_float()

        if system_type == "pendulum_lc":
            controller_path = params["controller_path"].as_string()
            controller_path = prx.lib_path + controller_path
            self.controller = torch.load(controller_path)
            self.controller.eval()
            torch.manual_seed(params["random_seed"].as_int())
        
        self.x_0_noise = None 
        self.f_noise = None 
        self.u_t_noise = None 
        self.t_noise = None

        if params["x_0_noise"].as_string() == "uniform":
            noise_params = params["x_0_noise_params"].as_float_vector()
            self.x_0_noise = prx.uniform_noise(
                noise_params[0], noise_params[1])
        
        # if params["f_noise"] == "uniform":
        #     noise_params = params["f_noise_params"].as_float_vector()
        #     self.f_noise = prx.uniform_noise(
        #         noise_params[0], noise_params[1])
        
        if params["u_t_noise"].as_string() == "uniform":
            noise_params = params["u_t_noise_params"].as_float_vector()
            self.u_t_noise = prx.uniform_noise(
                noise_params[0], noise_params[1])
        
        if params["t_noise"].as_string() == "uniform":
            noise_params = params["t_noise_params"].as_float_vector()
            self.t_noise = prx.uniform_noise(
                noise_params[0], noise_params[1])
    
    def pendulum_lc(self, X):
        self.ss.copy_point_from_vector(self.start_state,X)
        if self.x_0_noise is not None:
            self.x_0_noise.add_noise(self.start_state)
        self.ss.copy_from_point(self.start_state)
        self.ss.enforce_bounds()

        ctrl_input = torch.zeros(1, 4)

        duration_so_far = 0
        ctrl = [0]

        total_time = self.time_step
        if self.t_noise is not None:
            self.t_noise.add_noise(total_time)         

        while duration_so_far < total_time and prx.space_t.euclidean_2d(self.start_state, self.goal_state, 0, 2) > self.radius:
            ctrl_input[0, 0] = self.start_state[0]
            ctrl_input[0, 1] = self.start_state[1]
            ctrl_input[0, 2] = self.goal_state[0]
            ctrl_input[0, 3] = self.goal_state[1]

            with torch.no_grad():
                ctrl_output = self.controller(ctrl_input)[0].cpu()

            # ctrl = [-0.6371781908344007 + ((ctrl_output + 1.)*0.6371781908344007)]
            ctrl = np.array([-0.6371781908344007 + ((ctrl_output + 1.)
                                                    * 0.6371781908344007)], dtype=np.float64)

            self.ctrl[0] = ctrl[0]
            if self.u_t_noise is not None:
                self.u_t_noise.add_noise(self.ctrl)
            self.cs.copy_from_point(self.ctrl)
            self.cs.enforce_bounds()

            self.plant.propagate(self.simulation_step)
            self.ss.copy_to_point(self.start_state)

            duration_so_far += self.simulation_step

        self.ss.copy_to_point(self.end_state)
        return [self.end_state[0], self.end_state[1]]

    def pendulum_lqr(self, X):
        self.ss.copy_point_from_vector(self.start_state,X)
        if self.x_0_noise is not None:
            self.x_0_noise.add_noise(self.start_state)
        self.ss.copy_from_point(self.start_state)
        self.ss.enforce_bounds()

        if self.lqr == None:
            self.plant.linearize(self.goal_state, self.u_goal)
            self.Q = prx.matrix.Identity(2, 2)
            self.R = prx.matrix.Identity(1, 1)
            self.lqr = prx.lqr(self.plant, self.Q, self.R, "LQR")
            self.lqr.set_goal(self.goal_state)
            self.lqr.compute_K()
            self.K = self.lqr.get_K()
            #self.ps[1] = self.params["/plant/friction"].as_float()

        total_time = self.time_step
        if self.t_noise is not None:
            self.t_noise.add_noise(total_time) 
        duration_so_far = 0
        while duration_so_far <= total_time and prx.space_t.euclidean_2d(self.start_state, self.goal_state, 0, 2) > self.radius:
            self.lqr.compute_controls()
            # u_noise is TODO for LQR
            self.cs.enforce_bounds()
            self.plant.propagate(self.simulation_step)
            self.ss.copy_to_point(self.start_state)

            duration_so_far += self.simulation_step

        self.ss.copy_to_point(self.end_state)
        return [self.end_state[0], self.end_state[1]]