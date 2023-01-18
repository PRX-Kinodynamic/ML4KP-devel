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


if __name__ == "__main__":
    TM = NoisyTimeMap.NoisyTimeMap("examples/tripods/traj_tracking.yaml")
    TM.params.print()
    # lower_bounds = TM.ss.get_lower_bounds()
    # upper_bounds = TM.ss.get_upper_bounds()
    lower_bounds = TM.params["/plant/starting_lower_bound"].as_float_vector()
    upper_bounds = TM.params["/plant/ending_upper_bound"].as_float_vector()
    step_inc = TM.params["state_increment"].as_float()
    sys_name = TM.params["system_name"].as_string()

    # total_states = get_total_states(lower_bounds, upper_bounds, step_inc)
    # print("Total states:", total_states)

    # pt = TM.ss.make_point()
    # TM.ss.copy_point_from_vector(pt, lower_bounds)

    # roa_file_name = prx.out_path + TM.params["out_dir"].as_string() + "/" + sys_name + TM.params["file_name_suffix"].as_string()
    # fout_roa = open(roa_file_name, "w", buffering=2^10)
    # print("Output file: ", roa_file_name)

    # traj_file = prx.out_path + "pend_nominal_traj_2.0000000.000000.txt"
    # plan_file = prx.out_path + "pend_nominal_plan_2.0000000.000000.txt"
    # 
    TM.pendulum_trajectory_segment([0,0])
    TM.resulting_trajectory.to_file(prx.out_path + "/py_pend_track_trajs.txt")
    # for th in np.arange(-.25,.25,0.01):
    #     for thdot in np.arange(-.25,.25,0.01):
    #         TM.pendulum_trajectory_ilqr([th,thdot])
    #         TM.resulting_trajectory.to_file(prx.out_path + "/py_pend_track_trajs.txt", "a")
