import numpy as np 
import matplotlib.pyplot as plt 
import os
import sys
def load_data(fname):
	raw_data = np.genfromtxt(fname,delimiter=',',unpack=True)
	times, iters, costs = raw_data[4], raw_data[1], raw_data[3]
	return times, iters, costs

def analyze_quality(exps_dir,planner,max_time=10,num_trials=10):
	data = []
	for i in range(num_trials):
		run_data = []
		fname=exps_dir+"/"+planner+"/"+str(i)+".txt"
		times, _, costs = load_data(fname)
		for time, cost in zip(times,costs):
			if time > max_time: break
			if cost != 0.0:
				run_data.append([time,cost])
		data.append(run_data)

	out_x = []
	out_y = []
	k = 0

	current = [0 for i in range(num_trials)]
	final_sol = [0 for i in range(num_trials)]

	while k <= max_time:
		avg = 0
		num = 0
		for i in range(num_trials):
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
	out_x = np.vstack(out_x)
	out_y = np.vstack(out_y)
	return out_x, out_y

if __name__ == "__main__":
	# exps_dir = "/Users/aravind/Code/ML4KP-devel/out/1003"
	exps_dir = "/Users/aravind/Code/ML4KP-devel/out/results_segway_warehouse"
	planners = []
	for item in sorted(os.listdir(exps_dir)):
		if item[0] != '.': planners.append(item)
	markers = ["^","s","*","o",".","P"]

	plot_data_x = {}
	plot_data_y = {}
	for planner in planners:
		plot_data_x[planner], plot_data_y[planner] = analyze_quality(exps_dir,planner,60)
	
	plt.figure(figsize=(8,8))
	plt.rc('xtick',labelsize=16)
	plt.rc('ytick',labelsize=16)
	plt.grid()
	for i, planner in enumerate(planners):
		plt.plot(plot_data_x[planner],plot_data_y[planner],label=planner,marker=markers[i],markevery=10)
	plt.legend(loc="upper right",prop={'size':16})
	plt.title("Solution Quality",fontsize=16)
	plt.xlabel("Time (s)",fontsize=16)
	plt.ylabel("Cost",fontsize=16)
	plot_start = 0.0
	plt.xlim(plot_start,60)
	plt.show()