import numpy as np 
import os
from tqdm import tqdm

def f_sin(sim_time):
    return 10 * np.sin(0.5*sim_time)

def f_cos(sim_time):
    return 10 * np.cos(0.5*sim_time)

simulation_step = 0.1

data_dir = os.environ["DIRTMP_PATH"]+"out/dynamic/data/"
fname = "trajectory_10000.txt"
dat = np.loadtxt(data_dir+fname,delimiter=",")
print(dat.shape)

new_dat = np.zeros((dat.shape[0],17))

for i in tqdm(range(dat.shape[0])):
    start_time = dat[i,0]
    new_dat[i,:10] = dat[i,1:11]
    new_dat[i,10:12] = [-5.0,f_cos((start_time)*simulation_step)]
    new_dat[i,12:14] = [ 0.0,f_sin((start_time)*simulation_step)]
    new_dat[i,14:16] = [ 5.0,f_cos((start_time)*simulation_step)]
    new_dat[i,16]    = dat[i,-1]

np.savetxt(data_dir+"trajectory_10000_dynamic.txt",new_dat,delimiter=",")
