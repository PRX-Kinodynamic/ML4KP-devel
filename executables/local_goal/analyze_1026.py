import numpy as np 
import os

np.set_printoptions(precision=2)

exps_dir = os.environ["DIRTMP_PATH"] + "out/1026/medium/"

num_trials = 10
data = []
roadmap_time = []
roadmap_vertices = []
roadmap_edges = []
for problem in sorted(os.listdir(exps_dir)):
    if os.path.isdir(exps_dir+problem):
        success = []
        costs = []
        times = []
        iters = []
        branching = []
        full_fname = exps_dir+problem+"/"+"solution.txt"
        dat = np.loadtxt(full_fname,delimiter=',')
        roadmap_time.append(dat[4])
        v_fname = exps_dir+problem+"/"+"vertices.txt"
        e_fname = exps_dir+problem+"/"+"edges.txt"
        v_dat = np.loadtxt(v_fname,delimiter=',')
        e_dat = np.loadtxt(e_fname,delimiter=',')
        roadmap_vertices.append(v_dat.shape[0])
        roadmap_edges.append(e_dat.shape[0])
        if dat[0] != 0:
            success.append(1)
            costs.append(dat[0])
            times.append(dat[1])
            iters.append(dat[2])
            branching.append(dat[3])
        else:
            success.append(0)
        data.append([problem,np.mean(success),np.mean(costs),np.mean(times),np.mean(iters),np.mean(branching)])
        print(problem, np.mean(success), np.mean(costs), np.mean(times), np.mean(iters), np.mean(branching))

# Save data
data = np.array(data)
np.savetxt(exps_dir+"data.csv",data,delimiter=',',fmt="%s")
print("Roadmap time: ", np.mean(roadmap_time))
print("Roadmap vertices: ", np.mean(roadmap_vertices))
print("Roadmap edges: ", np.mean(roadmap_edges))