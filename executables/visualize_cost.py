import numpy as np 
import matplotlib.pyplot as plt 
import os
import sys
from collections import defaultdict

def load_data(fname):
	raw_data = np.genfromtxt(fname,delimiter=',',unpack=True)
	costs, times, iters = raw_data[3], raw_data[4], raw_data[5]
	return times, iters, costs

def get_best_cost_per_problem(exps_dir):
	best_cost = np.inf
	for f in os.listdir(exps_dir):
		if f.endswith(".txt"):
			_, _, costs = load_data(exps_dir+"/"+f)
			if costs[-1] != 0.0:
				best_cost = min(best_cost,costs[-1])
	return best_cost

def analyze_quality_time(exps_dir,problems,planner,best_costs):
	data = []
	max_time = 0
	total_trials = 0

	for problem in problems:
		current_dir = exps_dir+"/"+problem
		# Find number of files that start with planner_ inside exps_dir
		num_trials = len([name for name in os.listdir(current_dir) if os.path.isfile(os.path.join(current_dir, name)) and name.startswith(planner) and name.endswith(".txt")])
		total_trials += num_trials
		for i in range(num_trials):
			run_data = []
			fname=current_dir+"/"+planner+"_"+str(i)+".txt"
			times , _, costs = load_data(fname)
			max_time = max(max_time,max(times))
			for time, cost in zip(times,costs):
				if cost != 0.0:
					run_data.append([time,cost/best_costs[problem]])
			data.append(run_data)

	out_x = []
	out_y = []
	k = 0

	current = [0 for i in range(total_trials)]
	final_sol = [0 for i in range(total_trials)]

	while k <= max_time:
		avg = 0
		num = 0
		for i in range(total_trials):
			if len(data[i]) == 0: continue
			while data[i][current[i]][0] < k and current[i] < len(data[i])-1:
				current[i] += 1
			if k >= data[i][0][0]:
				avg += data[i][current[i]][1]
				num += 1
				final_sol[i] = data[i][current[i]][1]
		if avg != 0:
			out_x.append(k)
			avg /= float(num)
			out_y.append(avg)
		k += 1.0

	if len(out_x) == 0:
		return [], []	

	out_x = np.array(out_x)
	out_y = np.array(out_y)
	return out_x, out_y

if __name__ == "__main__":
	exps_dir = os.environ["DIRTMP_PATH"] + "out/ablation/indoors"
	planners = ["random","rlg"]
	markers = ["^","s","*","o",".","P"]
	problems = list(os.listdir(exps_dir))

	best_costs = {}
	for problem in problems:
		best_costs[problem] = get_best_cost_per_problem(exps_dir+"/"+problem)

	plot_data_x = {}
	plot_data_y = {}
	min_plot_x = np.inf
	labels = {}
	for planner in planners:
		labels[planner] = planner 
		# labels[planner].replace("roadmap_","")
		plot_data_x[planner], plot_data_y[planner] = analyze_quality_time(exps_dir,problems,planner,best_costs)
		min_plot_x = min(min_plot_x,plot_data_x[planner][-1])

	plt.figure(figsize=(8,8))
	plt.rc('xtick',labelsize=16)
	plt.rc('ytick',labelsize=16)
	plt.grid()
	for i, planner in enumerate(planners):
		# Remove the prefix from the label
		plt.plot(plot_data_x[planner],plot_data_y[planner],label=labels[planner],marker=markers[i],markevery=100)
	plt.legend(loc="upper right",prop={'size':16})
	plt.title("Normalized Solution Cost",fontsize=16)
	plt.xlabel("Time",fontsize=16)
	plt.ylabel("Cost",fontsize=16)
	#plt.xlim(0,min_plot_x)

	plots_dir = os.environ["DIRTMP_PATH"]+"out/success_rate.png"
	plt.savefig(plots_dir)
	#plt.show()