import sys
import math
import random
# Remember to add libpyDirtMP to your PYTHONPATH
# On bash: ``export PYTHONPATH=$DIRTMP_PATH/lib/:$PYTHONPATH
import libpyDirtMP as prx 
import numpy as np

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
	ps = plant.get_parameter_space()

	u_goal = cs.make_point()
	if plant_name == "pendulum":
		ps[1] = params["/plant/friction"].as_float()
		u_goal[0] = 0
	elif plant_name == "Acrobot":
		ps[0] = params["/plant/mass"].as_float()
		ps[1] = params["/plant/g"].as_float()
		u_goal[0] = 0


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

	# plant.linearize()
	plant.linearize(goal_state, u_goal)

	ss_dim = ss.get_dimension()
	cs_dim = cs.get_dimension()
	
	Q = prx.matrix.Identity(ss_dim, ss_dim);
	q_vec = params["/plant/lqr_Q"].as_float_vector();
	# Using the type matrix, but really is a vector
	v_goal = prx.vector.Zero(ss_dim);
	# v_goal = prx.matrix.Zero(ss_dim, 1);
	for i in range(ss_dim):
		Q[i,i] = q_vec[i]
		v_goal[i] = goal_state[i]
	print("Q:", Q)
	R = prx.matrix.Identity(cs_dim, cs_dim);
	R[0,0]=1
	# ss.copy_vector_from_point(v_goal, goal_state);
	print("goal_state:", goal_state)
	print("v_goal:", v_goal)
	lqr = prx.lqr(plant, Q, R, "LQR");
	lqr.set_goal(v_goal);
	lqr.compute_K();
	K = lqr.get_K();
	print("K: ", K);

	def G(X, tau, step, int_step=0.01, BOUNDS_ON_DOTS=6):
		# This hasn't changed
		ssmin = [0, -np.pi, -BOUNDS_ON_DOTS, -BOUNDS_ON_DOTS]
		ssmax = [2 * np.pi, np.pi, BOUNDS_ON_DOTS, BOUNDS_ON_DOTS]
		
		# Now, we can copy directly a python list/vector to a space
		ss.copy_from_vector(X)
		# Using python lists directly
		cs_lb = [-tau]
		cs_ub = [tau]

		# Assign bound to the space. Internally, it checks that the dimensions
		# of the space and the list are the same, throwing an error if not.
		ss.set_bounds(ssmin, ssmax);
		cs.set_bounds(cs_lb, cs_up);

		print("control \\in", cs.get_lower_bounds(), cs.get_upper_bounds())
		print("state \\in", ss.get_lower_bounds(), ss.get_upper_bounds())
		# construct to auxiliary points. This are not strictly necessary...
		ss_pt_aux = ss.make_point()
		cs_pt_aux = cs.make_point()
		# This is ok, but it might be better to use the condition_checker
		# with a while true: (...) => if checker.check(): break
		for i in range(step):
			# before it was: acrobot.apply_lqr()
			lqr.compute_controls() 
			# Redundant, this is already called internally but for safety is ok
			cs.enforce_bounds()  
			# Same as before
			plant.propagate(int_step)

			# Before there was a separate python function "check_bounds".
			# Now, we can use the space.satisfies_bounds(point):bool function
			# So, use the auxiliary points to copy from the space to the point
			ss.copy_to_point(ss_pt_aux)
			cs.copy_to_point(cs_pt_aux)
			# And assert that everything is as expected
			assert ss.satisfies_bounds(ss_pt_aux)
			assert cs.satisfies_bounds(cs_pt_aux)
			# print("state:", ss_pt_aux)
			# print("dist:", prx.space_t.euclidean_2d(ss_pt_aux, goal_state, 0, 4))
			if prx.space_t.euclidean_2d(ss_pt_aux, goal_state, 0, 4) < 0.1:
				print("Goal reached!")
				break
			# For visualization, copy the contents of the state space to the traj
			# If visualizations are not required, it might be better to skip this
			solution_traj.copy_onto_back(ss)
		# Return a list with the last state of the plant
		return [ss_pt_aux[0], ss_pt_aux[1], ss_pt_aux[2], ss_pt_aux[3]]
		# Alternatively, you could return the point directly, which is of type
		# prx.space_point and can be accesses similar to a list.
		# return ss_pt_aux

	# the start state could be assigned directly from the config file
	# start_state_vec = params["/plant/start_state"].as_float_vector()
	
	# Or from a python list
	# start_state_vec = [ 0.0, 0.0, 0.0, 0.0 ]
	print("G:", G(params["/plant/start_state"].as_float_vector(), 20, 500))

	# function G basically does the same as in here, with some extra stuff that
	# is done before (like assigning bounds).
	# print("control \\in", cs.get_lower_bounds(), cs.get_upper_bounds())
	# while True:
	# 	lqr.compute_controls()
	# 	cs.enforce_bounds()
	# 	plant.propagate(simulation_step)
	# #   # std::cout << "[plant] " << plant << std::endl;
	# 	solution_traj.copy_onto_back(ss);
	# 	if checker.check():
	# 		break

	vis_group = prx.three_js_group([plant], obstacle_list)

	body_name = params["/plant/name"].as_string() + "/" + params["/plant/vis_body"].as_string()

	vis_group.add_detailed_vis_infos(prx.info_geometry.FULL_LINE, solution_traj, body_name, ss, "0xFF0000")

	vis_group.add_animation(solution_traj, ss, start_state)

	params.print();
	vis_group.output_html("py_lqr_ctrl.html")

