import sys
import math
import random
import libpyDirtMP as prx
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
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

	vec_geoms = plant.get_geometries()
	vec_confs = plant.get_configurations()
	print(vec_geoms[0].name)
	print(vec_confs[0].name, vec_confs[0].transform)
	# for g in vec_geoms:
	# 	print("geoms:", g.name )
	# for g in vec_confs:
	# 	print("geoms:", g.name )
	# for state in solution_traj:
	# 	ss.copy_from_point(state)
	# 	plant.update_configuration()
	# 	confs = plant.get_configurations()
	# 	for c in confs:
	# 		if c.name == plant_name+"/ball":
	# 			tr = c.transform.translation()
	# 			print("state:", state, "conf:", tr[0], tr[1], tr[2])

	# fig = plt.figure()
	# ax = fig.add_subplot(111, autoscale_on=False, xlim=(-25, 25), ylim=(-25, 25))
	# ax.grid()

	# line, = ax.plot([], [], 'o-', lw=2)
	# time_template = 'time = %.1fs'
	# time_text = ax.text(0.05, 0.9, '', transform=ax.transAxes)


	# def init():
	# 	line.set_data([], [])
	# 	time_text.set_text('')
	# 	return line, time_text


	# def animate(i):
	# 	ss.copy_from_point(solution_traj[i.item()])
	# 	plant.update_configuration()
	# 	confs = plant.get_configurations()
	# 	for c in confs:
	# 		if c.name == plant_name+"/ball":
	# 			tr = c.transform.translation()
	# 			thisx = [0, tr[0]]
	# 			thisy = [0, tr[1]]
	# 			# print("state:", state, "conf:", tr[0], tr[1], tr[2])
	# 	# thisx = [0, x1[i], x2[i]]
	# 	# thisy = [0, y1[i], y2[i]]

	# 	line.set_data(thisx, thisy)
	# 	time_text.set_text(time_template % (i*simulation_step))
	# 	return line, time_text

	# ani = animation.FuncAnimation(fig, animate, np.arange(1, len(solution_traj)),
	# 		interval=25, blit=True, init_func=init)

	# plt.show()

	vis_group = prx.three_js_group([plant], obstacle_list)

	body_name = params["/plant/name"].as_string() + "/" + params["/plant/vis_body"].as_string()

	vis_group.add_detailed_vis_infos(prx.info_geometry.FULL_LINE, solution_traj, body_name, ss, "0xFF0000")

	vis_group.add_animation(solution_traj, ss, start_state)

	vis_group.output_html("py_no_control.html")

