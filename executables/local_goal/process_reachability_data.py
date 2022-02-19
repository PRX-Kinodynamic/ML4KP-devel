import numpy as np 
from tqdm import tqdm

data_dir = "/home/aravind/repos/ML4KP-devel/out/data_reach_treaded_vehicle/"
num_trajs = 10

for k in tqdm(range(num_trajs)):
    dat = np.loadtxt(data_dir+"trajectory_"+str(k)+".txt",delimiter=",")

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
    np.savetxt(data_dir+"processed_"+str(k)+".txt",tuples,delimiter=",")