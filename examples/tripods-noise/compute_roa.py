import io
import math
import numpy as np 
import NoisyTimeMap
from tqdm import tqdm
import libpyDirtMP as prx

def state_increment(space_point, step_inc, lower_bounds, upper_bounds):
    for i in range(len(space_point)):
        pt[i] = pt[i] + step_inc
        if pt[i] <= upper_bounds[i]:
            return True
        pt[i] = lower_bounds[i]
    return False;

def get_total_states(starting_lower_bound, ending_upper_bound, step_inc):
    total_states = 1
    for l,u in zip(starting_lower_bound, ending_upper_bound):
        total_states *= 1. + math.floor((u - l) / step_inc);
    return int(total_states);

def print_state(state):
    state_str = ""
    for s in state:
        state_str += str(s) + " "
    return state_str


if __name__ == "__main__":


    TM = NoisyTimeMap.NoisyTimeMap("examples/tripods/compute_roa.yaml")
    TM.params.print()
    lower_bounds = TM.ss.get_lower_bounds()
    upper_bounds = TM.ss.get_upper_bounds()
    step_inc = TM.params["state_increment"].as_float()
    sys_name = TM.params["system_name"].as_string()
    num_samples = TM.params["num_samples"].as_int()

    total_states = get_total_states(lower_bounds, upper_bounds, step_inc)
    print("Total states:", total_states)

    pt = TM.ss.make_point()
    TM.ss.copy_point_from_vector(pt, lower_bounds)

    roa_file_name = prx.out_path + TM.params["out_dir"].as_string() + "/" + sys_name + TM.params["file_name_suffix"].as_string()
    fout_roa = open(roa_file_name, "w", buffering=2^10)

    for _ in tqdm(range(total_states)):
        
        fout_roa.write(str(pt))
        reached = 0
        for x in range(num_samples):
            end_state_vec = TM.g_func(pt.to_list())

            # if TM.check_goal_reached(2): reached += 1
            if TM.goal_check(): reached += 1


        fout_roa.write(str(reached/num_samples)  + "\n")

        # fout_roa.write("\n")


        state_increment(pt, step_inc, lower_bounds, upper_bounds)
    fout_roa.close()