import numpy as np 
import os 

data_dir = os.environ["DIRTMP_PATH"]+"out/dynamic/prescience_test/"
num_trials = 30

# Success rate
successes = np.zeros((num_trials))
for i in range(num_trials):
    traj = np.loadtxt(data_dir+"trajectory_" + str(i) + ".txt",delimiter=",")
    if(traj.shape[0] == 0): continue
    if sum(traj[:,-1]) == traj.shape[0]: successes[i] = 1
print(successes)
num_successes = sum(successes)

# Average time taken to find the first solution
first_solution_time = []
for i in range(num_trials):
    if successes[i] != 1: continue 
    dat = np.loadtxt(data_dir+"dirt_" + str(i) + ".txt",delimiter=",")
    first_solution_time.append(min(dat[:,4]))

# Average cost of the first solution
first_solution_cost = []
for i in range(num_trials):
    if successes[i] != 1: continue 
    dat = np.loadtxt(data_dir+"dirt_" + str(i) + ".txt",delimiter=",")
    first_solution_cost.append(max(dat[:,3]))

# Average cost of the final solution
final_solution_cost = []
for i in range(num_trials):
    if successes[i] != 1: continue 
    dat = np.loadtxt(data_dir+"dirt_" + str(i) + ".txt",delimiter=",")
    final_solution_cost.append(dat[-1,3])

# Max iters across all trials
max_iters = -1
min_iters = np.inf
for i in range(num_trials):
    dat = np.loadtxt(data_dir+"dirt_" + str(i) + ".txt",delimiter=",")
    if max_iters < dat[-1,1]: max_iters = dat[-1,1]
    if min_iters > dat[-1,1]: min_iters = dat[-1,1]
    

print("Success rate:",num_successes/num_trials)
print("Average time to find the first solution: ",np.mean(first_solution_time))
print("Average cost of the first solution: ",np.mean(first_solution_cost))
print("Average cost of the final solution: ",np.mean(final_solution_cost))
print("Max iters: ",max_iters)
print("Min iters: ",min_iters)