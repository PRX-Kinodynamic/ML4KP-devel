import sys
import math
import random
import libpyDirtMP as prx


def state_increment(space_point, step_inc, lower_bounds, upper_bounds):
	for i in range(len(space_point)):
		pt[i] = pt[i] + step_inc
		if pt[i] <= upper_bounds[i]:
			return True
		pt[i] = lower_bounds[i]
	return False;

if __name__ == "__main__":
	params = prx.param_loader("examples/tripods/pendulum_lqr_roa.yaml", sys.argv);
	print(params.get_input_path())
	params.print()

	prx.set_simulation_step(params["simulation_step"].as_float())
	prx.init_random(params["random_seed"].as_int())

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

	cs_lb = params["/plant/control_space_lower_bound"].as_float_vector()
	cs_up = params["/plant/control_space_upper_bound"].as_float_vector()
	cs.set_bounds(cs_lb, cs_up);
	
	start_state = ss.make_point()
	goal_state  = ss.make_point()

	ss.copy_point_from_vector(start_state, params["/plant/start_state"].as_float_vector())
	ss.copy_point_from_vector(goal_state, params["/plant/goal_state"].as_float_vector())
	ss.copy_from_point(start_state)
	


	# solution_traj.copy_onto_back(ss)
	# plant.__class__ = prx.pendulum
	# print(repr(plant))
	plant.linearize()

	Q = prx.matrix.Identity(2,2)
	R = prx.matrix.Identity(1,1)
	print("Q:", Q)
	print("R:", R)

	lqr = prx.lqr(plant, Q, R, "LQR");
	lqr.compute_K();
	K = lqr.get_K();
	print("K:", K)

	roa_file_name = prx.lib_path + params["py_roa_file"].as_string();	
	fout_roa = open(roa_file_name, "w")

	traj_id = 0;
	rad = params["goal_region_radius"].as_float();
	def distance_function(a, b):
		return prx.space_t.euclidean_2d(a, b, 0, 2);
	
	def compute_traj(state):
		checker = prx.condition_check(params["checker_type"].as_string(), params["checker_value"].as_int())
		ss.copy_from_point(state)
		solution_traj = prx.trajectory(ss)

		while True:
			lqr.compute_controls()
			cs.enforce_bounds()
			plant.propagate(simulation_step)
			solution_traj.copy_onto_back(ss)
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
		compute_traj(pt)
		traj_id += 1
		# print(pt)
		if not state_increment(pt, 0.01, lower_bounds, upper_bounds):
			break

	print("Finished!")
