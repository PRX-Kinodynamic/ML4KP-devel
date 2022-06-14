import os 
import numpy as np 
import matplotlib.pyplot as plt
from tqdm import tqdm

data_dir = os.environ["DIRTMP_PATH"]+"out/dynamic/pred_test/"
num_dynamic_obstacles = 3
idx = num_dynamic_obstacles*2 + 1

avg_errors = {}
times = {}
plt.figure(figsize=(8,8))

planning_times = [0.1, 0.2, 0.5]
horizons = [0.5,1.0, 2.0, 5.0]

for pt in planning_times:
    for h in horizons:
        if pt < h: 
            gt_fname = data_dir + "ground_truth_" + str(pt) + "_" + str(h) + ".txt"
            pred_fname = data_dir + "predictions_" + str(pt) + "_" + str(h) + ".txt"
            gt_dat = np.loadtxt(gt_fname,delimiter=",")[:,:idx]
            pred_dat = np.loadtxt(pred_fname,delimiter=",")[:,:idx]
            errors = []
            for i in range(num_dynamic_obstacles):
                error = np.zeros((gt_dat.shape[0],))
                for j in range(gt_dat.shape[0]):
                    error[j] = np.linalg.norm(gt_dat[j,i*2+1:(i*2)+3] - pred_dat[j,i*2+1:(i*2)+3])
                errors.append(error)

            errors = np.vstack(errors)
            avg_error = np.mean(errors,axis=0)
            avg_errors[str(pt)+"_"+str(h)] = avg_error
            times[str(pt)+"_"+str(h)] = gt_dat[:,0]

            plt.grid()
            plt.xlim(0,gt_dat[-1,0])
            plt.plot(gt_dat[:,0],avg_error,label='Average')
            for i, error in enumerate(errors):
                plt.plot(gt_dat[:,0],error,label='Box '+str(i))
            plt.legend(loc='best')
            plt.title("Predictive model accuracy for planning time = "+str(pt)+" and horizon = "+str(h))
            plt.xlabel("Time (s)")
            plt.ylabel("Error")
            plt.savefig(data_dir+"error_"+str(pt)+"_"+str(h)+".png")
            plt.clf()

thresh = 2.0
plt.grid()
plt.xlim(0,gt_dat[-1,0])
plt.ylim(0,thresh)
for key, val in avg_errors.items():
    if np.max(val) <= thresh:
        plt.plot(times[key],val,label=key)
plt.legend(loc='best')
plt.title("Runs with average error <= "+str(thresh))
plt.xlabel("Time (s)")
plt.ylabel("Error")
plt.savefig(data_dir+"error_avg.png")