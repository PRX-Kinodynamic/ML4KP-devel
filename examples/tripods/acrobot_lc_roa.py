import libpyDirtMP as prx
import numpy as np
import torch
from tqdm import tqdm
import TimeMap
np.set_printoptions(suppress=True)

if __name__ == "__main__":
    step = 4  # for grid
    time_h = 10  # in seconds, time_h / simulation_step = total steps

    # Provide the path to the controller

    TM = TimeMap.TimeMap("acrobot_lc", time_h,
                         "examples/tripods/acrobot_lc.yaml")

    def g(X):
        return TM.acrobot_lc(X)

    ts = np.linspace(0, 2*np.pi, step)
    bs = np.linspace(-np.pi, np.pi, step)
    cs = np.linspace(-6, 6, step)
    ds = np.linspace(-6,6,step)
    line = ""

    start_state = TM.ss.make_point()
    end_state = TM.ss.make_point()

    print(TM.time_step)

    for a in tqdm(range(ts.shape[0])):
        for b in range(bs.shape[0]):
            for c in range(cs.shape[0]):
                for d in range(ds.shape[0]):
                    start_state_vec = [ts[a], bs[b], cs[c], ds[d]]
                    TM.ss.copy_point_from_vector(start_state, start_state_vec)

                    end_state_vec = g(start_state_vec)

                    TM.ss.copy_point_from_vector(end_state, end_state_vec)

                    line += str(start_state) + str(end_state) + \
                        str(prx.space_t.euclidean_2d(end_state, TM.goal_state, 0, 4) < 0.6) + "\n"

    name_file = f"acrobot_lc_low_{step}_{time_h}.out"
    with open(name_file, "w") as f:
        f.write(line)

    # plot_traj = []
    # for s in solution_traj:
    #     plot_traj.append(s.to_list())
    # plot_traj = np.vstack(plot_traj)
    # print(plot_traj[-1])
    # plt.figure(figsize=(8,8))
    # plt.xlim(-10,10)
    # plt.ylim(-10,10)
    # plt.plot(plot_traj[:,0],plot_traj[:,1],color='black')
    # plt.title("Cost = "+str(counter))
    # plt.show()
