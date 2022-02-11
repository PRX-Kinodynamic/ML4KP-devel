import torch
import numpy as np
import libpyDirtMP as prx
import TimeMap as tm
# from tqdm import tqdm

def line_to_state(line, range_states, state, safe, safe_index):

    s_i = 0
    tokens = line.split()
    for i in range(range_states[0], range_states[1]+1):
        state[s_i] = float(tokens[i])
        s_i += 1
    return int(tokens[safe_index]) == 1

def get_end_state(map_fn, pt):
    return map_fn(pt.to_list())

def df(a, TM):
    return prx.space_t.euclidean_2d(a, TM.goal_state, 0, TM.ss.get_dimension())

TM = None
if __name__ == "__main__":
    # params = prx.param_loader("examples/tripods/ackermann_ha_roa.yaml", sys.argv);

    file_name = "/home/gary/motion_planning/ML4KP-devel/out/ackermann_FO/roa_00101.txt"; 
    yaml_file = "examples/tripods/ackermann_ha_roa.yaml"
    tm_plant_name = "ackermann_lqr"
    safe_index = 3
    state_ids = [0,2]
    map_fn = lambda x: TM.ackermann_lqr(x)


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


    TM = tm.TimeMap(tm_plant_name, checker_value * simulation_step, params)


    TM.ss.copy_point_from_vector(TM.goal_state, goal_state_vec)

    print("goal_state:", TM.goal_state)
    true_pos = 0;
    true_neg = 0;
    false_pos = 0;
    false_neg = 0;
    reached = False;
    safe = False;
    total_states = 0;


    state = TM.ss.make_point()
    end_state = TM.ss.make_point()

    f_in = open(file_name, "r");
    # while (std::getline(infile, line))
    for line in f_in:
    
        # print("line: ",line);
        # std::istringstream iss(line);
        # double th, thdot, safe_val;
        # toks = line.split();

        safe = line_to_state(line, state_ids, state, safe, safe_index)

        ev = get_end_state(map_fn, state);
        TM.ss.copy_point_from_vector(end_state, ev);

        reached = df(end_state, TM) <= 0.1;
        # print("state:", state)
        # print("end_state:", end_state)
        # print("reached:", reached, "safe", safe)
        true_pos  = true_pos  + int(  reached     &   safe     );
        true_neg  = true_neg  + int((not reached) & (not safe) );
        false_pos = false_pos + int((not reached) &   safe     );
        false_neg = false_neg + int(  reached     & (not safe) );

        total_states += 1;

        # print("state: ", state);
        # if total_states > 10000: 
        #     break

    print("Total states: ", total_states )
    print("true_pos: ",  true_pos,  "\t---\t", 100 * true_pos  / total_states )
    print("true_neg: ",  true_neg,  "\t---\t", 100 * true_neg  / total_states )
    print("false_pos: ", false_pos, "\t---\t", 100 * false_pos / total_states )
    print("false_neg: ", false_neg, "\t---\t", 100 * false_neg / total_states )
