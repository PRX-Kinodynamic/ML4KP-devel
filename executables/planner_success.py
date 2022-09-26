import numpy as np 
import matplotlib.pyplot as plt 
import os
import sys

def load_data(fname):
	raw_data = np.genfromtxt(fname,delimiter=',',unpack=True)
	times, iters, costs = raw_data[4], raw_data[1], raw_data[3]
	return times, iters, costs

def analyze_success_rates(exps_dir,planner,max_time=10,num_trials=10):
	data = []
	for i in range(num_trials):
		fname=exps_dir+"/"+planner+"/"+str(i)+".txt"
		times, _, costs = load_data(fname)
		for time, cost in zip(times,costs):
			if time > max_time: break
			if cost == 0.0:
				data.append([time,0,i])
			else:
				data.append([time,1,i])

	data.sort()
	solved = []
	for j in range(num_trials):
		for d in data:
			if d[2] == j and d[1] == 1:
				solved.append(d[0])
				break

	solved.sort()
	print(planner,len(solved))
	out_y = []
	out_x = []
	num_solved = 0
	k = 0
	print_flag = False
	while k <= max_time:
		out_x.append(k)
		if num_solved != len(solved) and k >= solved[num_solved]:
			num_solved += 1
		out_y.append(1.0*num_solved/(num_trials))
		k += 1e-2
		if k >= 0.2 and print_flag:
			print(out_y[-1])
			print_flag = False
	return out_x, out_y

if __name__ == "__main__":
	exps_dir = "/Users/aravind/Code/ML4KP-devel/out/results_segway/"
	planners = []
	for item in sorted(os.listdir(exps_dir)):
		if item[0] != '.': planners.append(item)
	markers = ["^","s","*","o",".","P"]

	plot_data_x = {}
	plot_data_y = {}
	for planner in planners:
		plot_data_x[planner], plot_data_y[planner] = analyze_success_rates(exps_dir,planner,60)

	plt.figure(figsize=(8,8))
	plt.rc('xtick',labelsize=16)
	plt.rc('ytick',labelsize=16)
	plt.grid()
	for i, planner in enumerate(planners):
		plt.plot(plot_data_x[planner],plot_data_y[planner],label=planner,marker=markers[i],markevery=1000)
	plt.legend(loc="lower right",prop={'size':16})
	plt.xlabel("Time (s)",fontsize=16)
	plt.ylabel("Success Rate",fontsize=16)
	plt.title("Success Rate",fontsize=16)
	plt.ylim(0,1)
	plt.xlim(0,60)
	plt.show()