import numpy as np 

data_dir = "/home/aravind/repos/ML4KP-devel/out/data_reach_treaded_vehicle/"
num_trajs = 1

dat = np.loadtxt(data_dir+"trajectory_0.txt",delimiter=",")

tuples = []

for i in range(dat.shape[0]):
    if dat[i,-1] == 0:
        for j in range(i+1,dat.shape[0]):
            tuples.append(np.hstack([dat[i,:-1],dat[j,:-1],0]))
    else:
        for j in range(i+1,dat.shape[0]):
            if dat[j,-1] == 1:
                tuples.append(np.hstack([dat[i,:-1],dat[j,:-1],1]))
            else: break

tuples = np.vstack(tuples)
np.savetxt(data_dir+"processed_0.txt",tuples,delimiter=",")