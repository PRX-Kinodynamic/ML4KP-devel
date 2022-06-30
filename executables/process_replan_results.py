import numpy as np 
import os
from tqdm import tqdm

goal = np.array([9.0,0.0])
goal_radius = 0.1
# num_trials = 30
num_trials = 10

exps_dir = os.environ["DIRTMP_PATH"] + "out/dynamic/no_prescience_unsafe"
dirs = sorted(os.listdir(exps_dir))


for i in range(len(dirs)):
    successes = 0.0
    solution_costs = []
    for j in range(num_trials):
        traj = np.loadtxt(exps_dir+"/"+dirs[i]+"/trajectory_"+str(j)+".txt",delimiter=",")
        final_state = traj[-1]
        if((np.linalg.norm(final_state[:2] - goal) < goal_radius) and \
             (final_state[-1] == 1)):
            successes += 1
            solution_costs.append(0.1*(len(traj)-1))
    print(dirs[i]+"_"+f"{successes/num_trials:.2f}"
            +f"_{np.mean(solution_costs):.2f}")