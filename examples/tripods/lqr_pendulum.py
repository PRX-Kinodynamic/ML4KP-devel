import sys
import math
import random
# Remember to add libpyDirtMP to your PYTHONPATH
# On bash: ``export PYTHONPATH=$DIRTMP_PATH/lib/:$PYTHONPATH
import libpyDirtMP as prx 

if __name__ == "__main__":

	# This is able to handle *.yaml files as well as command line arguments
	# in the form of --argument=value and --/arg_root/arg_l1/arg_l2=value
	# i.e.: 
	# 	Simple example: python lqr_pendulum.py --checker_value=5  
	# 	Complex (with vectors!): python lqr_pendulum.py --/plant/start_state=\[1,-1.0]
	# 	More complex: python lqr_pendulum.py --checker_value=5  --/plant/start_state=\[1,-1.0]
	params = prx.param_loader("examples/intermediate/lqr.yaml", sys.argv);
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
	
	checker = prx.condition_check(params["checker_type"].as_string(), params["checker_value"].as_int())

	solution_traj = prx.trajectory(ss)

	solution_traj.copy_onto_back(ss)
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
	# simulating a do{}while()
	print("start_state:", start_state)
	while True:
		lqr.compute_controls();
		cs.enforce_bounds();
		plant.propagate(simulation_step);
	# 	# std::cout << "[pendulum] " << plant << std::endl;
		solution_traj.copy_onto_back(ss);
		if checker.check():
			break

	vis_group = prx.three_js_group([plant], obstacle_list)

	body_name = params["/plant/name"].as_string() + "/" + params["/plant/vis_body"].as_string()

	vis_group.add_detailed_vis_infos(prx.info_geometry.FULL_LINE, solution_traj, body_name, ss, "0xFF0000")

	vis_group.add_animation(solution_traj, ss, start_state)

	vis_group.output_html("py_lqr_pendulum_control.html")