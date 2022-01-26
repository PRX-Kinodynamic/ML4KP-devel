import numpy as np 
import TimeMap as tm
import libpyDirtMP as prx


def state_increment(space_point, step_inc, lower_bounds, upper_bounds):
	for i in range(len(space_point)):
		pt[i] = pt[i] + step_inc
		if pt[i] <= upper_bounds[i]:
			return True
		pt[i] = lower_bounds[i]
	return False;


yaml_file = "examples/tripods/ackermann_ha_roa.yaml"
params = prx.param_loader(yaml_file)

step_inc = params["state_increment"].as_float();

lower_bounds = params["/plant/state_space_lower_bound"].as_float_vector()
upper_bounds = params["/plant/state_space_upper_bound"].as_float_vector()
goal_state_vec = params["/plant/goal_state"].as_float_vector()

checker_value = params["checker_value"].as_float()
simulation_step = params["simulation_step"].as_float()
rad = params["goal_region_radius"].as_float()
time = checker_value * simulation_step
plant_name = params["/plant/name"].as_string()
TM = tm.TimeMap("ackermann_hyb", checker_value * simulation_step, params)

pt = TM.ss.make_point()
end_state = TM.ss.make_point()
goal_state = TM.ss.make_point()
goal_state_mid = TM.ss.make_point()

TM.ss.copy_point_from_vector(goal_state, goal_state_vec)
TM.ss.copy_point_from_vector(goal_state_mid, [-10, 0, 0])
# 1.75 8 -0.391593

roa_file_name = prx.lib_path + "out/ackermann_FO/hybpr_" + plant_name + "_roa.txt"; 
print("ROA file:", roa_file_name)
fout_roa = open(roa_file_name, "w")

starting_lower_bound = params["/plant/starting_lower_bound"].as_float_vector()
ending_upper_bound = params["/plant/ending_upper_bound"].as_float_vector()

TM.ss.copy_point_from_vector(pt, starting_lower_bound)

TM.k_rho_1     = +1.0
TM.k_alpha_1   = +5.0
TM.k_beta_1    = -3.0

TM.goal_state = goal_state
while True:
	# print(pt)
	line = str(pt) + " "
	# TM.goal_state = goal_state_mid
	end_state_v = TM.ackermann_hyb(pt.to_list())
	# TM.goal_state = goal_state
	# end_state_v = TM.ackermann_hyb(end_state_v)
	TM.ss.copy_point_from_vector(end_state, end_state_v)
	
	reached = 0

	if prx.space_t.euclidean_2d(goal_state, end_state, 0, 3) <= rad: reached = 1
	line += str(reached) + "\n"
	
	fout_roa.write(line)


	if not state_increment(pt, step_inc, starting_lower_bound, ending_upper_bound):
		break


