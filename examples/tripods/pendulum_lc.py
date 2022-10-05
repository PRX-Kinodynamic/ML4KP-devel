import sys
import torch
import numpy as np 
import libpyDirtMP as prx

# Provide the path to the controller
if __name__ == "__main__":

	params = prx.param_loader("examples/tripods/pendulum_lc.yaml", sys.argv);

	path_to_model = prx.lib_path + params["controller_path"].as_string()
	controller = torch.jit.load(path_to_model)
	controller.eval()
	prx.init_random(params["random_seed"].as_int())
	torch.manual_seed(params["random_seed"].as_int())

	simulation_step = params["simulation_step"].as_float()
	prx.set_simulation_step(simulation_step)

	plant_name = params["/plant/name"].as_string()
	plant_path = params["/plant/path"].as_string()

	plant = prx.system_factory.create_system(plant_name, plant_path);

	wm = prx.world_model([plant], [])
	wm.create_context("context", [plant_name], [])
	context = wm.get_context("context");

	ss = context.system_group.get_state_space()
	cs = context.system_group.get_control_space()

	lower_bounds = params["/plant/state_space_lower_bound"].as_float_vector()
	upper_bounds = params["/plant/state_space_upper_bound"].as_float_vector()

	ss.set_bounds(lower_bounds, upper_bounds)

	start_state = ss.make_point()
	goal_state  = ss.make_point()

	start_state_vec = params["/plant/start_state"].as_float_vector()
	goal_state_vec = params["/plant/goal_state"].as_float_vector()

	ss.copy_point_from_vector(start_state, start_state_vec)
	ss.copy_point_from_vector(goal_state,  goal_state_vec)
	ss.copy_from_point(start_state)
	
	checker = prx.condition_check(params["checker_type"].as_string(), params["checker_value"].as_int())

	state = ss.make_point()
	ctrl_pt = cs.make_point()
	
	solution_traj = prx.trajectory(ss);

	ss.copy_point(state, start_state);

	current = start_state_vec

	ctrl_input = torch.zeros(1,4)

	checker = prx.condition_check(params["checker_type"].as_string(), params["checker_value"].as_float())

# while( counter < 100 and prx.space_t.euclidean_2d(state, goal_state, 0, 2) > 0.1):
	solution_traj.copy_onto_back(ss)
	while True:

		# ss.copy_vector_from_point(current,state)
		current = state.to_list()
		ctrl_input[0,0] = current[0]
		ctrl_input[0,1] = current[1]
		ctrl_input[0,2] = goal_state[0]
		ctrl_input[0,3] = goal_state[1]

		with torch.no_grad():
			controller_out = controller(ctrl_input)[0].cpu()
			# controller_out = controller(torch.Tensor(current + goal_state_vec))[0].cpu()
		ctrl = np.array([-0.6371781908344007+ ((controller_out[0] + 1.)*0.6371781908344007)], dtype=np.float64)
		# print("ctrl:", ctrl)
		cs.copy_from_vector(ctrl)

		cs.enforce_bounds();
		plant.propagate(simulation_step);
		solution_traj.copy_onto_back(ss)
		ss.copy_to_point(state);
		# if checker.check() or prx.space_t.euclidean_2d(state, goal_state, 0, 2) < 0.01:
		if checker.check():
			print("final state:", state)
			break

	vis_group = prx.three_js_group([plant], [])

	body_name = params["/plant/name"].as_string() + "/" + params["/plant/vis_body"].as_string()

	vis_group.add_detailed_vis_infos(prx.info_geometry.FULL_LINE, solution_traj, body_name, ss, "0xFF0000")

	vis_group.add_animation(solution_traj, ss, start_state)

	vis_group.output_html("py_pendulum_lc_control.html")