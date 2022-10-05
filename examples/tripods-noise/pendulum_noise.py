import libpyDirtMP as prx
import numpy as np 
from tqdm import tqdm
import NoisyTimeMap


if __name__ == "__main__":
    step = 11
    time_h = 5

    # TM = NoisyTimeMap.NoisyTimeMap("pendulum_lc", time_h,
                                #    "examples/tripods/pendulum_lc_noise.yaml")
    g_name = "pendulum_lqr"
    g_name = "pendulum_lc"
    TM = NoisyTimeMap.NoisyTimeMap("examples/tripods/pendulum_noise.yaml")

    def g(X):
        # return TM.pendulum_lc(X)
        return TM.pendulum_lqr(X)
    
    xs = np.linspace(-np.pi, np.pi, step)
    ys = np.linspace(-2*np.pi, 2*np.pi, step)

    start_state = TM.ss.make_point()
    end_state = TM.ss.make_point()

    line = ""
    for a in tqdm(range(xs.shape[0])):
        for b in range(ys.shape[0]):
            start_state_vec = [xs[a], ys[b]]
            TM.ss.copy_point_from_vector(start_state, start_state_vec)

            # end_state_vec = g(start_state_vec)
            end_state_vec = TM.g_func(start_state_vec)

            TM.ss.copy_point_from_vector(end_state, end_state_vec)

            line += str(end_state) + "\n"
    
    name_file = f"/home/gary/motion_planning/ML4KP-devel/out/pendulum_noise.out"
    with open(name_file, "w") as f:
        f.write(line)

