import sys
import math
import random
import libpyDirtMP as prx

if __name__ == "__main__":
	params = prx.param_loader("examples/basic/no_control.yaml", sys.argv);
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

	ss.copy_point_from_vector(start_state, params["/plant/start_state"].as_float_vector())
	ss.copy_from_point(start_state)

	solution_traj = prx.trajectory(ss)
	solution_traj.copy_onto_back(ss)

	checker = prx.condition_check(params["checker_type"].as_string(), params["checker_value"].as_int())

	while True:
		cs.enforce_bounds();
		plant.propagate(simulation_step);
		# std::cout << "[pendulum] " << plant << std::endl;
		solution_traj.copy_onto_back(ss);
		if checker.check():
			break

	vis_group = prx.three_js_group([plant], obstacle_list)

	body_name = params["/plant/name"].as_string() + "/" + params["/plant/vis_body"].as_string()

	vis_group.add_detailed_vis_infos(prx.info_geometry.FULL_LINE, solution_traj, body_name, ss, "0xFF0000")

	vis_group.add_animation(solution_traj, ss, start_state)

	vis_group.output_html("py_no_control.html")

