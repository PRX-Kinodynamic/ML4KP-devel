import libpyDirtMP as prx 
import numpy as np 
from tqdm import tqdm
import TimeMap
np.set_printoptions(suppress=True)

if __name__ == "__main__":
    fname = "acrobot_lc_low_50_10.out"
    dat = np.loadtxt(fname)
    # @Ewerton: we can do this in batches for the LQR.
    starts = dat[:,4:-1]
    time_h = 100

    TM = TimeMap.TimeMap("acrobot_lqr",time_h,"examples/tripods/acrobot_roa.yaml")
    line = ""

    start_state = TM.ss.make_point()
    end_state = TM.ss.make_point()

    def g(X):
        return TM.acrobot_lqr(X)
    
    for i in tqdm(range(starts.shape[0])):
        start_state_vec = starts[i]
        TM.ss.copy_point_from_vector(start_state,start_state_vec)

        end_state_vec, tt = g(start_state_vec)
        TM.ss.copy_point_from_vector(end_state,end_state_vec)

        line += str(start_state) + str(end_state) + \
            str(prx.space_t.euclidean_2d(end_state,TM.goal_state,0,4)<0.1) + " " + str(tt) + "\n"
    
    name_file = f"acrobot_lqr_100.out"
    with open(name_file, "w") as f:
        f.write(line)