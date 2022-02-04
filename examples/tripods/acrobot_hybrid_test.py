import libpyDirtMP as prx 
import numpy as np 
import TimeMap
from tqdm import tqdm
np.set_printoptions(suppress=True)

if __name__ == "__main__":
    step = 4
    time_h = 10 

    TM_lqr = TimeMap.TimeMap("acrobot_lqr",time_h,"examples/tripods/acrobot_roa.yaml")
    TM_lc  = TimeMap.TimeMap("acrobot_lc" ,time_h,"examples/tripods/acrobot_lc.yaml")
    line = ""

    def g(X):
        output_of_lc = TM_lc.acrobot_lc(X)
        print(output_of_lc)
        return TM_lqr.acrobot_lqr(output_of_lc)
    
    ts = np.linspace(0, 2*np.pi, step)
    bs = np.linspace(-np.pi, np.pi, step)
    cs = np.linspace(-6, 6, step)
    ds = np.linspace(-6,6,step)
    line = ""

    start_state = TM_lc.ss.make_point()
    end_state = TM_lqr.ss.make_point()

    for a in tqdm(range(ts.shape[0])):
        for b in range(bs.shape[0]):
            for c in range(cs.shape[0]):
                for d in range(ds.shape[0]):
                    start_state_vec = [ts[a], bs[b], cs[c], ds[d]]
                    TM_lc.ss.copy_point_from_vector(start_state, start_state_vec)

                    end_state_vec = g(start_state_vec)

                    TM_lqr.ss.copy_point_from_vector(end_state, end_state_vec)

                    line += str(start_state) + str(end_state) + \
                        str(prx.space_t.euclidean_2d(end_state, TM_lc.goal_state, 0, 4) < 0.1) + "\n"

    name_file = f"acrobot_hybrid_10.out"
    with open(name_file, "w") as f:
        f.write(line)
