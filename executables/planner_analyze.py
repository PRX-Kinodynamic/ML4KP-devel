import numpy as np 
import os

def load_data(fname):
	raw_data = np.genfromtxt(fname,delimiter=',',unpack=True)
	times, iters, costs = raw_data[4], raw_data[5], raw_data[3]
	return times, iters, costs

if __name__ == "__main__":
    exps_dir = "/Users/aravind/Code/ML4KP-devel/out/test_results"
    max_time = 10
    planners = []
    for item in sorted(os.listdir(exps_dir)):
        if item[0] != '.': planners.append(item)
    
    for planner in planners:
        print(planner)
        first_solution_times = []
        first_solution_costs = []
        first_solution_iters = []
        final_solution_costs = []

        for i in range(10):
            fname=exps_dir+"/"+planner+"/"+str(i)+".txt"
            times, iters, costs = load_data(fname)
            # Find first non-zero cost
            for j in range(iters.shape[0]):
                if costs[j] != 0.0:
                    first_solution_times.append(times[j])
                    first_solution_costs.append(costs[j])
                    first_solution_iters.append(iters[j])
                    break
            
            # Find final cost
            for j in range(times.shape[0]-1,-1,-1):
                if times[j] <= 10.0 and costs[j] != 0.0:
                    final_solution_costs.append(costs[j])
                    break
        
        print("First solution times: ", np.mean(first_solution_times))
        print("First solution costs: ", np.mean(first_solution_costs))
        print("First solution iters: ", np.mean(first_solution_iters))
        print("Final solution costs: ", np.mean(final_solution_costs))
        print("Success rate: ", len(final_solution_costs)/10.0)
            
