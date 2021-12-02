import sys
import math
import random
# Remember to add libpyDirtMP to your PYTHONPATH
# On bash: ``export PYTHONPATH=$DIRTMP_PATH/lib/:$PYTHONPATH
import libpyDirtMP as prx 


if __name__ == "__main__":

	params = prx.param_loader("examples/intermediate/lqr.yaml", sys.argv);

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

	checker = prx.condition_check(params["checker_type"].as_string(), params["checker_value"].as_int())

	solution_traj = prx.trajectory(ss)
	solution_traj.copy_onto_back(ss)

	plant.linearize()

	ss_dim = ss.get_dimension()
	cs_dim = cs.get_dimension()
	
	Q = prx.matrix.Identity(ss_dim, ss_dim);
	q_vec = params["/plant/lqr_Q"].as_float_vector();
	# Using the type matrix, but really is a vector
	v_goal = prx.vector.Zero(ss_dim);
	# v_goal = prx.matrix.Zero(ss_dim, 1);
	for i in range(ss_dim):
		Q[i,i] = q_vec[i]
		# v_goal[i,0] = goal_state[i]
	print("Q:", Q)
	R = prx.matrix.Identity(cs_dim, cs_dim);

	ss.copy_vector_from_point(v_goal, goal_state);
	lqr = prx.lqr(plant, Q, R, "LQR");
	lqr.set_goal(v_goal);
	lqr.compute_K();
	K = lqr.get_K();
	print("K: ", K);
	
	while True:
		lqr.compute_controls()
		cs.enforce_bounds()
		plant.propagate(simulation_step)
	#   # std::cout << "[plant] " << plant << std::endl;
		solution_traj.copy_onto_back(ss);
		if checker.check():
			break

	vis_group = prx.three_js_group([plant], obstacle_list)

	body_name = params["/plant/name"].as_string() + "/" + params["/plant/vis_body"].as_string()

	vis_group.add_detailed_vis_infos(prx.info_geometry.FULL_LINE, solution_traj, body_name, ss, "0xFF0000")

	vis_group.add_animation(solution_traj, ss, start_state)

	params.print();
	vis_group.output_html("py_lqr_ctrl.html")

