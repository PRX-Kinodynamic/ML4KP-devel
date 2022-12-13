import numpy as np 
import os

np.set_printoptions(precision=2)

exps_dir = os.environ["DIRTMP_PATH"] + "out/1212/car/"

planners = ["random","rlg","roadmap"]
data = []
for planner in planners:
    for problem in sorted(os.listdir(exps_dir)):
        if os.path.isdir(exps_dir+problem):
            success = []
            costs = []
            times = []
            iters = []
            branching = []
            if planner == "roadmap":
                num_trials = 1
            else:
                num_trials = 10
            for i in range(num_trials):
                    full_fname = exps_dir+problem+"/"+planner+"_"+str(i)+".txt"
                    # Check if file exists
                    if os.path.isfile(full_fname):
                        dat = np.loadtxt(full_fname,delimiter=',')
                        if dat[0] != 0:
                            success.append(1)
                            costs.append(dat[0])
                            times.append(dat[1])
                            iters.append(dat[2])
                            branching.append(dat[3])
                        else:
                            success.append(0)
            data.append([planner,problem,np.mean(success),np.mean(costs),np.mean(times),np.mean(iters),np.mean(branching)])
            print(planner,problem, np.mean(success), np.mean(costs), np.mean(times), np.mean(iters), np.mean(branching))

# Save data
data = np.array(data)
np.savetxt(exps_dir+"data.csv",data,delimiter=',',fmt="%s")