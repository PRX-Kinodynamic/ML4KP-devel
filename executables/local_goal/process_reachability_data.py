import numpy as np 
from tqdm import tqdm

data_dir = "/home/kushal/ML4KP-devel/out/data_reach_treaded_vehicle/"
num_trajs = 10000

tuples = []
for k in tqdm(range(num_trajs)):
    dat = np.loadtxt(data_dir+"trajectory_"+str(k)+".txt",delimiter=",")
    for i in range(dat.shape[0]):
        if dat[i,-1] == 0:
            for j in range(i+1,dat.shape[0]):
                tuples.append(np.hstack([dat[i,:-1],dat[j,:-1],0]))
        else:
            for j in range(i+1,dat.shape[0]):
                if dat[j,-1] == 1:
                    tuples.append(np.hstack([dat[i,:-1],dat[j,:-1],1]))
                else: 
                    for k in range(j, dat.shape[0]):
                        tuples.append(np.hstack([dat[i,:-1],dat[k,:-1],0]))
                    break

tuples = np.vstack(tuples)
np.savetxt("processed_treaded.txt",tuples,delimiter=",")