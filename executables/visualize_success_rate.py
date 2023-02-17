import numpy as np 
import matplotlib.pyplot as plt 
import os
import sys

def load_data(fname):
	raw_data = np.genfromtxt(fname,delimiter=',',unpack=True)
	costs, times, iters = raw_data[3], raw_data[4], raw_data[5]
	return times, iters, costs

def analyze_success_time(exps_dir,problems,planner):
    data = []
    max_time = 0
    total_trials = 0

    for problem in problems:
        current_dir = exps_dir+"/"+problem
        # Find number of files that start with planner_ inside exps_dir
        num_trials = len([name for name in os.listdir(current_dir) if os.path.isfile(os.path.join(current_dir, name)) and name.startswith(planner) and name.endswith(".txt")])
        for i in range(num_trials):
            run_data = []
            fname=current_dir+"/"+planner+"_"+str(i)+".txt"
            times, _, costs = load_data(fname)
            max_time = max(max_time,max(times))
            for time, cost in zip(times,costs):
                if cost == 0.0:
                    data.append([time,0,i+total_trials])
                else:
                    data.append([time,1,i+total_trials])
        total_trials += num_trials
    
    data.sort()
    solved = []
    for j in range(total_trials):
        for d in data:
            if d[2] == j and d[1] == 1:
                solved.append(d[0])
                break

    solved.sort()
    out_y = []
    out_x = []
    num_solved = 0
    k = 0

    while k <= max_time:
        out_x.append(k)
        if num_solved != len(solved) and k >= solved[num_solved]:
            num_solved += 1
        out_y.append(1.0*num_solved/(total_trials))
        k += 1
    
    return out_x, out_y

if __name__ == "__main__":
    exps_dir = os.environ["DIRTMP_PATH"] + "out/ablation/indoors"
    planners = ["random","rlg"]
    markers = ["^","s","*","o",".","P"]
    problems = list(os.listdir(exps_dir))

    plot_data_x = {}
    plot_data_y = {}
    labels = {}
    for planner in planners:
        labels[planner] = planner
        plot_data_x[planner], plot_data_y[planner] = analyze_success_time(exps_dir,problems,planner)

    plt.figure(figsize=(8,8))
    plt.rc('xtick',labelsize=16)
    plt.rc('ytick',labelsize=16)
    plt.grid()
    for i, planner in enumerate(planners):
        plt.plot(plot_data_x[planner],plot_data_y[planner],label=labels[planner],marker=markers[i],markevery=100)
    plt.legend(loc="lower right",prop={'size': 16})
    plt.title("Success Rate",fontsize=16)
    plt.xlabel("Time",fontsize=16)
    plt.ylabel("Success Rate",fontsize=16)
    plt.xlim(0,300)
    plt.ylim(0,1.0)
    plt.show()