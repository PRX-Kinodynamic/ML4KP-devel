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
	path_to_model = prx.lib_path + params["controller_path"].as_string()
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
	current     = ss.make_point()

	ss.copy_point_from_vector(start_state, params["/plant/start_state"].as_float_vector())
	ss.copy_point_from_vector(goal_state, params["/plant/goal_state"].as_float_vector())
	ss.copy_from_point(start_state)
	
	roa_file_name = prx.lib_path + params["py_roa_file"].as_string();	
	fout_roa = open(roa_file_name, "w")

	traj_id = 0;
	rad = params["goal_region_radius"].as_float();

	ctrl_input = torch.zeros(1,4)

	def distance_function(a, b):
		return prx.space_t.euclidean_2d(a, b, 0, 2);
	
	def compute_traj(state):
		checker = prx.condition_check(params["checker_type"].as_string(), params["checker_value"].as_int())
		ss.copy_from_point(state)
		solution_traj = prx.trajectory(ss)
		ss.copy_point(current, state)
		# current = state.to_list()
		while True:
			ctrl_input[0,0] = current[0]
			ctrl_input[0,1] = current[1]
			ctrl_input[0,2] = goal_state[0]
			ctrl_input[0,3] = goal_state[1]
			with torch.no_grad():
				controller_out = controller(ctrl_input)[0].cpu()
			ctrl = np.array([-0.6371781908344007+ ((controller_out[0] + 1.)*0.6371781908344007)], dtype=np.float64)
			cs.copy_from_vector(ctrl)

			cs.enforce_bounds()
			plant.propagate(simulation_step)
			solution_traj.copy_onto_back(ss)
			ss.copy_to_point(current);
			if checker.check():
				break

		end_state = solution_traj.back()
		line = str(traj_id) + " "
		line += str(state) + " "
		reached = 0
		if distance_function(end_state, goal_state) <= rad: reached = 1
		line += str(reached) + "\n"

		fout_roa.write(line)

	pt = ss.make_point()
	ss.copy_point_from_vector(pt, lower_bounds)

	while True:
		# print(pt)
		compute_traj(pt)
		traj_id += 1
		if not state_increment(pt, 0.01, lower_bounds, upper_bounds):
			break

	print("Finished!")
