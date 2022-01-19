import sys
import math
import torch
import random
import numpy as np 
import libpyDirtMP as prx
from scipy.spatial.transform import Rotation as R

def state_increment(space_point, step_inc, lower_bounds, upper_bounds):
	for i in range(len(space_point)):
		pt[i] = pt[i] + step_inc
		if pt[i] <= upper_bounds[i]:
			return True
		pt[i] = lower_bounds[i]
	return False;

print_once = True
def compute_controls(start_state, goal_state, ctrl1, ctrl2):
	global print_once
	r = R.from_euler('z', -np.pi/2.0 - goal_state[2], degrees=False)
	v1 = [-1.9, -1.8, -1.57]
	v2 = [ 0  ,  1.2, -1.57]

	v1 = r.apply(v1)
	v2 = r.apply(v2)

	if print_once:
		print("v1:", v1)
		print("v2:", v2)
		print_once = False
	if (v1[0] <= start_state[0] and
		v1[1] <= start_state[1] <= v2[1]):
		ctrl2.compute_controls()
	else:
		ctrl1.compute_controls();

# if -1.9 <= X[0] and -1.8 <= X[1] <= 1.:
    #     k_rho = 1
    #     k_alpha = 9.5
    #     k_beta = -9.5
    # else:
    #     k_rho = 1
    #     k_alpha = 9.5
    #     k_beta = -9


if __name__ == "__main__":
	params = prx.param_loader("examples/tripods/ackermann_ha_roa.yaml", sys.argv);

	simulation_step = params["simulation_step"].as_float()
	prx.set_simulation_step(simulation_step)
	prx.init_random(params["random_seed"].as_int())

	obstacles = prx.load_obstacles(params["environment"].as_string())
	obstacle_list = obstacles.objects
	obstacle_names = obstacles.names

	plant_name = params["/plant/name"].as_string();
	plant_path = params["/plant/path"].as_string();
	plant = prx.system_factory.create_system(plant_name, plant_path);
	if plant == None: 
		print("Error: plant not found!")
		exit(-1)

	wm = prx.world_model([plant], obstacle_list)
	wm.create_context("context", [plant_name], obstacle_names)
	context = wm.get_context("context")

	ss = context.system_group.get_state_space();
	cs = context.system_group.get_control_space();

	lower_bounds = params["/plant/state_space_lower_bound"].as_float_vector()
	upper_bounds = params["/plant/state_space_upper_bound"].as_float_vector()
	ss.set_bounds(lower_bounds, upper_bounds);

	cs_lb = params["/plant/control_space_lower_bound"].as_float_vector()
	cs_up = params["/plant/control_space_upper_bound"].as_float_vector()
	cs.set_bounds(cs_lb, cs_up);

	start_state = ss.make_point();
	goal_state = ss.make_point();
	end_state   = ss.make_point()

	ss.copy_point_from_vector(start_state, params["/plant/start_state"].as_float_vector())
	ss.copy_point_from_vector(goal_state, params["/plant/goal_state"].as_float_vector())
	ss.copy_from_point(start_state)
	
	checker = prx.condition_check(params["checker_type"].as_string(), params["checker_value"].as_int());

	solution_traj = prx.trajectory(ss);

	ss_dim = ss.get_dimension();
	cs_dim = cs.get_dimension();

	ss.print_bounds();
	cs.print_bounds();

	ctrl_1 = prx.ackermann_FO_ctrl(plant, "ackermann_FO_ctrl_1");
	ctrl_2 = prx.ackermann_FO_ctrl(plant, "ackermann_FO_ctrl_2");

	k_rho_1   = +1.0
	k_alpha_1 = +5.0
	k_beta_1  = -2.0

	k_rho_2   = +1.0
	k_alpha_2 = +5.0
	k_beta_2  = -9.5
	
	# ctrl_1.set_gains(k_rho_1, k_alpha_1, k_beta_1);
	# ctrl_2.set_gains(k_rho_2, k_alpha_2, k_beta_2);

	ctrl_1.set_gains(k_rho_1, k_alpha_1, k_beta_1);
	ctrl_2.set_gains(k_rho_2, k_alpha_2, k_beta_2);

	ctrl_1.set_goal(goal_state);
	ctrl_2.set_goal(goal_state);

	solution_traj.copy_onto_back(ss);

	while True:

		# if (-1.9 <= end_state[0] and
		# 	-1.8 <= end_state[1] and end_state[1] <= 1.2):
		# 	ctrl_2.compute_controls();
		# else:
		# 	ctrl_1.compute_controls();
		compute_controls(start_state, goal_state, ctrl_1, ctrl_2)
		# compute_controls(end_state, goal_state, ctrl_1, ctrl_2)
		cs.enforce_bounds();

		plant.propagate(simulation_step);
		ss.copy_to_point(end_state);

		# Remove this line when doing intensive computation
		solution_traj.copy_onto_back(end_state);

		if checker.check():
			break
	params.print();

	print("End state:", end_state)

	vis_group = prx.three_js_group([plant], [])

	body_name = params["/plant/name"].as_string() + "/" + params["/plant/vis_body"].as_string()

	vis_group.add_detailed_vis_infos(prx.info_geometry.FULL_LINE, solution_traj, body_name, ss, "0xFF0000")

	vis_group.add_animation(solution_traj, ss, start_state)

	vis_group.output_html("hybA_ackermann.html")
