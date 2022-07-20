import sys 
import os 
import torch 
import libpyDirtMP as prx 
import numpy as np 

# from inspect import currentframe, getframeinfo

class NoisyTimeMap:

    def g_func(self, X):
        return getattr(self, self.system_name, self.not_supported)(X)

    def not_supported(self, X):
        print("System: ", self.system_name, " not supported!")
        exit(-1);

    def __init__(self, parameters):
        if isinstance(parameters, str):
            params = prx.param_loader(parameters, sys.argv)
        elif isinstance(parameters, prx.param_loader):
            print("Instance of prx.param_loader")
            params = parameters
        
        self.duration = params["duration"].as_float() 
        self.time_step = self.duration # For backwards comp, should delete it eventually
        self.system_name = params["system_name"].as_string()
        self.params = params
        self.Q = None
        self.R = None
        self.K = None
        self.controller = None
        self.noisy_plant = None

        self.simulation_step = params["simulation_step"].as_float()
        prx.set_simulation_step(self.simulation_step)
        prx.init_random(params["random_seed"].as_int())

        self.obstacles = prx.obstacle_loader(params["environment"].as_string())

        plant_name = params["/plant/name"].as_string()
        plant_path = params["/plant/path"].as_string()
        self.plant = prx.system_factory.create_system(plant_name, plant_path)
        if self.plant == None:
            print("Error: plant not found!")
            exit(-1)

        self.wm = prx.world_model([self.plant], self.obstacles.get_obstacles())
        self.wm.create_context("context", [plant_name], self.obstacles.get_names())
        self.context = self.wm.get_context("context")

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
        self.traj = prx.trajectory(self.ss)

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

        self.x_0_noise = None 
        self.f_noise = None 
        self.u_t_noise = None 
        self.t_noise = None
        self.in_collision_py = lambda : self.context.collision_group.in_collision()

        self.checker = prx.condition_check("sim_time" , self.duration );
        self.goal_check = prx.create_default_goal_check(self.ss, self.goal_state, params["goal_region_radius"].as_float() );
        self.obstacle_check = prx.custom_check.wrap(self.in_collision_py );
        self.checker_gc = prx.condition_check( self.goal_check );
        self.checker_obstacle = prx.condition_check( self.obstacle_check );
        self.checker.add_condition(self.checker_gc);
        self.checker.add_condition(self.checker_obstacle);

        self.x_0_noise = self.init_noise("/plant/x_0_noise", "/plant/x_0_noise_params");
        self.u_t_noise = self.init_noise("/plant/u_t_noise", "/plant/u_t_noise_params");
        self.t_noise   = self.init_noise("/plant/t_noise",   "/plant/t_noise_params"  );
        self.f_noise   = self.init_noise("/plant/f_noise",   "/plant/f_noise_params"  );


    def init_noise(self, noise_type_pn, noise_params_pn):
        prx_noise = None
        noise_type = self.params[noise_type_pn].as_string()
        if noise_type == "uniform":
            noise_params = self.params[noise_params_pn].as_float_vector()
            prx_noise = prx.uniform_noise(
                noise_params[0], noise_params[1])
        else:
            print("Noise: ", noise_type, " not supported")
        return prx_noise

    def get_noisy_system(self):
        noise_type = self.params["/plant/f_noise"].as_string()
        if noise_type == "uniform":
            noise_params = self.params["/plant/f_noise_params"].as_float_vector()
            self.noisy_plant = prx.uniform_noisy_plant(self.plant, noise_params[0], noise_params[1]);
        elif noise_type == "None":
            self.noisy_plant = self.plant
        else:
            print("Noise: ", noise_type, " not supported")
            exit(-1)

    def get_noisy_controller(self):
        noise_type = self.params["/plant/u_t_noise"].as_string()
        if noise_type == "uniform":
            noise_params = self.params["/plant/u_t_noise_params"].as_float_vector()
            self.controller = prx.noisy_uniform_controller(self.controller_base, noise_params[0], noise_params[1]);
        elif noise_type == "None":
            self.controller = self.controller_base
        else:
            print("Noise: ", noise_type, " not supported")
            exit(-1)

    def check_goal_reached(self, dim):
        return prx.space_t.euclidean_2d(self.start_state, self.goal_state, 0, dim) <= self.radius

    def pendulum_lc(self, X):

        if self.controller == None:
            controller_path = self.params["/plant/controller_path"].as_string()
            controller_path = prx.lib_path + controller_path
            self.controller = torch.load(controller_path)
            self.controller.eval()
            torch.manual_seed(self.params["random_seed"].as_int())

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

        while duration_so_far < total_time and not self.check_goal_reached(2):
            if self.f_noise is not None:
                self.f_noise.add_noise(self.start_state)
            
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
        return self.end_state.to_list()
    

    def pendulum_lqr(self, X):

        self.ss.copy_point_from_vector(self.start_state,X)
        if self.x_0_noise is not None:
            self.x_0_noise.add_noise(self.start_state)
        self.ss.copy_from_point(self.start_state)
        self.ss.enforce_bounds()

        if self.noisy_plant == None:
            self.get_noisy_system()

        if self.controller == None:
            self.Q = prx.matrix.Identity(2, 2)
            self.R = prx.matrix.Identity(1, 1)
            self.controller_base = prx.lqr(self.noisy_plant, self.Q, self.R, "LQR")
            self.controller_base.set_goal(self.goal_state, self.u_goal)
            # self.controller_base.set_goal(self.goal_state)
            self.controller_base.compute_K()
            self.get_noisy_controller()
            # if self.u_t_noise is not None:
            #     self.controller = prx.noisy_uniform_controller
  
        total_time = self.duration
        if self.t_noise is not None:
            total_time = self.t_noise.add_noise(total_time) 

        self.checker.set_check_value(total_time)
        self.checker.reset()
        # print("Before propagate: ", self.start_state)
        self.context.system_group.propagate(self.start_state, self.controller, self.checker, self.end_state);
        # print("After propagate: ", self.end_state)
        return self.end_state.to_list()

    def pendulum_tbc(self, X):

        self.ss.copy_point_from_vector(self.start_state,X)
        if self.x_0_noise is not None:
            self.x_0_noise.add_noise(self.start_state)
        self.ss.copy_from_point(self.start_state)
        self.ss.enforce_bounds()

        if self.noisy_plant == None:
            self.get_noisy_system()

        if self.controller == None:
            u_min = self.cs.get_lower_bound(0);
            u_equ = 0;
            u_max = self.cs.get_upper_bound(0);

            self.set_of_ctrls = [[u_min, u_equ, u_max]];
            self.controller = prx.bang_bang(self.noisy_plant, self.set_of_ctrls, "bang_bang")
            self.controller.set_control(0)

            self.fout_roa = open(prx.out_path + self.params["out_dir"].as_string() + "/" + self.params["system_name"].as_string() + "_traj" + self.params["file_name_suffix"].as_string(), "w", buffering=2^10)

            # filename = "/Users/Gary/Downloads/pend_TBC_ctrl.csv"
            filename = prx.input_path + "/pend_TBC_ctrl.csv"
            self.ctrl_dict = {}
            with open(filename, 'r') as opened_file:
                for line in opened_file:
                    line_as_str = list(map(float, line.split(' ')))
                    box = (line_as_str[2], line_as_str[3], line_as_str[4], line_as_str[5])
                    self.ctrl_dict[box] = line_as_str[6]


  
        total_time = self.duration
        if self.t_noise is not None:
            total_time = self.t_noise.add_noise(total_time) 



        self.checker.set_check_value(total_time)
        self.checker.reset()
        
        self.traj.clear()

        self.traj.copy_onto_back(self.ss)
        while True:

            ctrl_num = 2
            for box in self.ctrl_dict:
                x_l = box[0]
                y_l = box[1]
                x_u = box[2]
                y_u = box[3]

                if x_l <= float(self.traj.back()[0]) < x_u:
                    if y_l <= float(self.traj.back()[1]) <= y_u:
                        ctrl_num = int(self.ctrl_dict[box])
                        break
            self.controller.set_control(ctrl_num)
            self.controller.compute_controls()

            self.plant.propagate(self.simulation_step)
            # self.ss.copy_to_point(self.start_state)
            self.traj.copy_onto_back(self.ss)

            if self.checker.check():
                break;

        past_state = self.traj[0]
        for state in self.traj:
            if prx.space_t.euclidean_2d(past_state, state) < 1:
                self.fout_roa.write(str(state) + "\n")
                past_state = state
            else:
                self.fout_roa.write("\n")
                past_state = state
        self.fout_roa.write("\n")
        return self.traj.back().to_list() 

    def pendulum_bang_bang(self, X, ctrl_num = 2):

        self.ss.copy_point_from_vector(self.start_state,X)
        if self.x_0_noise is not None:
            self.x_0_noise.add_noise(self.start_state)
        self.ss.copy_from_point(self.start_state)
        self.ss.enforce_bounds()

        if self.noisy_plant == None:
            self.get_noisy_system()

        if self.controller == None:
            u_min = self.cs.get_lower_bound(0);
            u_equ = 0;
            u_max = self.cs.get_upper_bound(0);

            self.set_of_ctrls = [[u_min, u_equ, u_max]];
            self.controller_base = prx.bang_bang(self.noisy_plant, self.set_of_ctrls, "bang_bang")
            self.controller_base.set_control(ctrl_num)
            self.get_noisy_controller()

            self.fout_roa = open(prx.out_path + self.params["out_dir"].as_string() + "/" + self.params["system_name"].as_string() + "_traj" + self.params["file_name_suffix"].as_string(), "w", buffering=2^10)

  
        total_time = self.duration
        if self.t_noise is not None:
            total_time = self.t_noise.add_noise(total_time) 

        self.checker.set_check_value(total_time)
        self.checker.reset()
        # print("Before propagate: ", self.start_state)
        # self.context.system_group.propagate(self.start_state, self.controller, self.checker, self.end_state);
        
        self.traj.clear()
        self.context.system_group.propagate(self.start_state, self.controller, self.checker, self.traj);

        # past_state = self.traj[0]
        # for state in self.traj:
        #     if prx.space_t.euclidean_2d(past_state, state) < 1:
        #         self.fout_roa.write(str(state) + "\n")
        #         past_state = state
        #     else:
        #         self.fout_roa.write("\n")
        #         past_state = state
        # self.fout_roa.write("\n")

        # print("After propagate: ", self.end_state)
        return self.end_state.to_list()

    def lander_analytical(self, X):
        self.ss.copy_point_from_vector(self.start_state,X)
        if self.x_0_noise is not None:
            self.x_0_noise.add_noise(self.start_state)
        self.ss.copy_from_point(self.start_state)
        self.ss.enforce_bounds()

        if self.noisy_plant == None:
            self.get_noisy_system()

        if self.controller == None:

            # self.lander_custom_check_1 = lambda : -0.1 <= self.end_state[0] <= 0.1
            # self.goal_check = lambda : -0.1 <= self.end_state[0] <= 0.1 and -0.5 <= self.end_state[1] <= 0.5
            def lander_custom_check_1():
                # print("Start:", self.start_state,"\tEnd:", self.end_state)
                # print(getframeinfo(currentframe()).filename, getframeinfo(currentframe()).lineno)
                return self.ss.at(0) <= -1 

            def lander_custom_check_2():
                # print("2) Start:", self.start_state,"\tEnd:", self.end_state)
                return -0.1 <= self.end_state[0] <= 0.1 and -0.5 <= self.end_state[1] <= 0.5

            self.lander_custom_check_1 = lander_custom_check_1
            # self.lander_custom_check_1 = lander_custom_check_2
            self.goal_check  = lander_custom_check_2

            self.checker = prx.condition_check("sim_time" , self.duration );
            # self.goal_check = prx.custom_check.wrap(lander_custom_check_2);
            # self.goal_check = lander_custom_check_2
            self.goal_check_2 = prx.custom_check.wrap(self.lander_custom_check_1);
            # self.goal_check_2 = prx.custom_check.wrap(self.lander_custom_check_2);
            self.checker_gc = prx.condition_check( self.goal_check_2 );
            self.checker.add_condition(self.checker_gc);

            
            self.controller_base = prx.lander_meditch_ctrl(self.noisy_plant, "lander_ctrl")
            self.get_noisy_controller()

        total_time = self.duration
        if self.t_noise is not None:
            total_time = self.t_noise.add_noise(total_time) 

        self.checker.set_check_value(total_time)
        self.checker.reset()
        self.context.system_group.propagate(self.start_state, self.controller, self.checker, self.end_state);
        # print("1) Start:", self.start_state,"\tEnd:", self.end_state)
        return self.end_state.to_list()