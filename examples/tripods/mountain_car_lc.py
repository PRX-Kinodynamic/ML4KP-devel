import libpyDirtMP as prx
import numpy as np 
import TimeMap 
from tqdm import tqdm

if __name__ == "__main__":
    step = 11
    time_h = 200

    TM = TimeMap.TimeMap("mountain_car_lc",time_h,
                        "examples/tripods/mountain_car_lc.yaml")
    
    def g(X):
        return TM.mountain_car_lc(X)

    xs = np.linspace(-1.2,0.6,11)
    ys = np.linspace(-.07,.07,11)
    line = ""

    start_state = TM.ss.make_point()
    end_state = TM.ss.make_point()

    for a in tqdm(range(xs.shape[0])):
        for b in range(ys.shape[0]):
            start_state_vec = [xs[a], ys[b]]
            TM.ss.copy_point_from_vector(start_state, start_state_vec)

            end_state_vec = g(start_state_vec)

            TM.ss.copy_point_from_vector(end_state, end_state_vec)

            line += str(start_state) + str(end_state) + \
                str(end_state[0] >= 0.6) + "\n"
    
    fname = f"mountain_car.out"
    with open(fname,"w") as f:
        f.write(line)
