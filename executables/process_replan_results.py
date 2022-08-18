import numpy as np 
import os
from tqdm import tqdm

# goal = np.array([9.0,0.0])
# goal_radius = 0.1
# num_trials = 30

goal = np.array([9.0,0.0,0.0,0.0,0.0])
goal_radius = 0.5
num_trials = 10

exps_dir = os.environ["DIRTMP_PATH"] + "out/dynamic/evaluation/replan/"
dirs = sorted(os.listdir(exps_dir))
avg_dists = []
success_rates = []

for i in range(len(dirs)):
    dirs[i] = exps_dir + dirs[i] + "/"
    successes = 0.0
    min_dist = np.inf
    min_dists = []
    for j in range(num_trials):
        traj = np.loadtxt(dirs[i]+"trajectory_"+str(j)+".txt",delimiter=",")
        for k in range(traj.shape[0]):
            dist = np.linalg.norm(traj[k,:2]-goal[:2])
            if dist < min_dist:
                min_dist = dist
        final_state = traj[-1,:-1]
        # if np.linalg.norm(final_state-goal) < goal_radius and sum(traj[:,-1]) == traj.shape[0]:
        if np.linalg.norm(final_state[:2]-goal[:2]) < goal_radius and sum(traj[:,-1]) == traj.shape[0]:
            successes += 1.0
        min_dists.append(min_dist)
    print(dirs[i],successes/num_trials,np.mean(min_dists))
    avg_dists.append(np.mean(min_dists))
    success_rates.append(successes/num_trials)

print(np.mean(avg_dists))
print(np.mean(success_rates))


# for i in range(len(dirs)):
#     successes = 0.0
#     solution_costs = []
#     for j in range(num_trials):
#         traj = np.loadtxt(exps_dir+"/"+dirs[i]+"/trajectory_"+str(j)+".txt",delimiter=",")
#         final_state = traj[-1]
#         # print((np.linalg.norm(final_state[:2] - goal)))
#         if((np.linalg.norm(final_state[:2] - goal) < goal_radius) and \
#              (final_state[-1] == 1)):
#             successes += 1
#             solution_costs.append(0.1*(len(traj)-1))
#     print(dirs[i]+"_"+f"{successes/num_trials:.2f}"
#             +f"_{np.mean(solution_costs):.2f}")