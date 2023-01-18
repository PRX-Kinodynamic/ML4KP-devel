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

    TM.pendulum_trajectory_segment([0,0])
    TM.resulting_trajectory.to_file(prx.out_path + "/py_pend_track_trajs.txt")
    for th in np.arange(-.25,.25,0.01):
        for thdot in np.arange(-.25,.25,0.01):
            TM.pendulum_trajectory_segment([th,thdot])
            TM.resulting_trajectory.to_file(prx.out_path + "/py_pend_track_trajs.txt", "a")
