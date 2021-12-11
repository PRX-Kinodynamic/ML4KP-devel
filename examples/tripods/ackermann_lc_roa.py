import sys
import math
import torch
import random
import numpy as np 
import libpyDirtMP as prx


def state_increment(space_point, step_inc, lower_bounds, upper_bounds):
	for i in range(len(space_point)):
		pt[i] = pt[i] + step_inc
		if pt[i] <= upper_bounds[i]:
			return True
		pt[i] = lower_bounds[i]
	return False;

if __name__ == "__main__":
	params = prx.param_loader("examples/tripods/lc_roa.yaml", sys.argv);
	path_to_model = prx.lib_path + params["/plant/controller_path"].as_string()
	controller = torch.jit.load(path_to_model)
	controller.eval()

	params.print()

	prx.set_simulation_step(params["simulation_step"].as_float())
	prx.init_random(params["random_seed"].as_int())
	torch.manual_seed(params["random_seed"].as_int())

	# Could be done directly from params, calling the getter to show that is there...
	simulation_step = prx.get_simulation_step()
	# Safety check
	print("simulation_step:", simulation_step)

	obstacles = prx.load_obstacles(params["environment"].as_string())
	obstacle_list = obstacles.objects
	obstacle_names = obstacles.names

	plant_name = params["/plant/name"].as_string()
	plant_path = params["/plant/path"].as_string()
	plant = prx.system_factory.create_system(plant_name, plant_path);
	# plant = prx.pendulum(plant_name, plant_path)
	# prx_assert(plant != nullptr, "Plant is nullptr!");
	if plant == None: 
		print("Error: plant not found!")
		exit(-1)

	wm = prx.world_model([plant], obstacle_list)
	wm.create_context("context", [plant_name], obstacle_names)
	context = wm.get_context("context")

	ss = context.system_group.get_state_space()
	cs = context.system_group.get_control_space()

	lower_bounds = params["/plant/state_space_lower_bound"].as_float_vector()
	upper_bounds = params["/plant/state_space_upper_bound"].as_float_vector()
	ss.set_bounds(lower_bounds, upper_bounds)

	start_state = ss.make_point()
	goal_state  = ss.make_point()
	end_state   = ss.make_point()

	ss.copy_point_from_vector(start_state, params["/plant/start_state"].as_float_vector())
	ss.copy_point_from_vector(goal_state, params["/plant/goal_state"].as_float_vector())
	ss.copy_from_point(start_state)
	
	out_dir = params["/plant/out_dir"].as_string() 
	file_id = params["/plant/file_id"].as_int()
	ss_file_id = str(file_id).zfill(5)
	roa_file_name = prx.lib_path + out_dir + "/lc_" + plant_name + "_" + ss_file_id + "_roa.txt";
	fout_roa = open(roa_file_name, "w")

	traj_id = 0;
	rad = params["goal_region_radius"].as_float();


	ss_dim = ss.get_dimension()

	starting_lower_bound = params["/plant/starting_lower_bound"].as_float_vector();
	ending_upper_bound = params["/plant/ending_upper_bound"].as_float_vector();

	pt = ss.make_point()
	ss.copy_point_from_vector(pt, starting_lower_bound)
	print("first pt:", pt)
	step_inc = params["state_increment"].as_float();

	total_states = 1
	for l,u in zip(starting_lower_bound, ending_upper_bound):
		total_states *= 1. + math.floor((u - l) / step_inc)
	print("Total states:", total_states);

	propagation_step = params["/plant/propagation_step"].as_float()

	bn = 25
	ctrl_input = torch.zeros(bn,6)

	def distance_function(a, b):
		return prx.space_t.euclidean_2d(a, b, 0, ss_dim);
	
	def compute_traj(state):
		global end_state
		checker = prx.condition_check(params["checker_type"].as_string(), params["checker_value"].as_int())
		ss.copy_from_point(state)
		ss.copy_point(end_state, state)
		# current = state.to_list()
		while True:

			for i in range(bn):
				ctrl_input[i,0] = end_state[0]
				ctrl_input[i,1] = end_state[1]
				ctrl_input[i,2] = end_state[2]
				ctrl_input[i,3] = goal_state[0]
				ctrl_input[i,4] = goal_state[1]
				ctrl_input[i,5] = goal_state[2]

			with torch.no_grad():
				ctrl_output = controller(ctrl_input).cpu()
			# print(ctrl_output)
			ctrl_output_mean = [0,0]
			for i in range(bn):
				ctrl_output_mean[0] += 1.0*ctrl_output[i, 0].item()
				ctrl_output_mean[1] += 1.0*ctrl_output[i, 1].item()
			ctrl_output_mean[0] = ctrl_output_mean[0]/bn
			ctrl_output_mean[1] = ctrl_output_mean[1]/bn

			ctrl = np.array([-np.pi/3 + ((ctrl_output_mean[0] + 1)*np.pi/3), (ctrl_output_mean[1]+1)*15], dtype=np.float64)
			
			cs.copy_from_vector(ctrl)

			cs.enforce_bounds()
			plant.propagate(propagation_step)
			ss.copy_to_point(end_state);
			if checker.check():
				break

		line = str(state) + " "
		reached = 0
		if distance_function(end_state, goal_state) <= rad: reached = 1
		line += str(reached) + "\n"

		fout_roa.write(line)

	ss.copy_point_from_vector(pt, starting_lower_bound)

	prev_last_dim = pt[ss_dim-1];

	while True:
		# print(pt)
		if (pt[ss_dim-1] != prev_last_dim):
			file_id += 1
			fout_roa.close();
			ss_file_id = str(file_id).zfill(5)
			roa_file_name = prx.lib_path + out_dir + "/lc_" + plant_name + "_" + ss_file_id + "_roa.txt";
			fout_roa = open(roa_file_name, "w")
			prev_last_dim = pt[ss_dim-1]

		compute_traj(pt)
		traj_id += 1
		if not state_increment(pt, step_inc, starting_lower_bound, ending_upper_bound):
			break

	print("Finished!")
