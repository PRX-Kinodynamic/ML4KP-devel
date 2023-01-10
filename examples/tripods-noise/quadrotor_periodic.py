import libpyDirtMP as prx
import numpy as np 
from tqdm import tqdm
import NoisyTimeMap


if __name__ == "__main__":
    step = 11
    time_h = 15

    # TM = NoisyTimeMap.NoisyTimeMap("pendulum_lc", time_h,
                                #    "examples/tripods/pendulum_lc_noise.yaml")
    g_name = "1D_Quadrotor"
    TM = NoisyTimeMap.NoisyTimeMap("examples/tripods/quadrotor.yaml")

    def g(X):
        return TM.quadrotor_lqr(X)
    
    xs = np.linspace(0, 20, step)
    ys = np.linspace(-2, 20, step)

    start_state = TM.ss.make_point()
    end_state = TM.ss.make_point()

    line = ""
    for a in tqdm(range(xs.shape[0])):
        for b in range(ys.shape[0]):
            start_state_vec = [xs[a], ys[b]]
            TM.ss.copy_point_from_vector(start_state, start_state_vec)

            end_state_vec = TM.g_func(start_state_vec)

            TM.ss.copy_point_from_vector(end_state, end_state_vec)

            line += str(end_state) + "\n"
    
    print(line)

