import numpy as np
with open('all_stats/total_timing_stats.txt', 'r') as file:
    lines = file.readlines()

data = []
for line in lines:
    d = []
    for x in line.split(','):
        d.append(float(x.strip()))
    if d[-1] == 0:
        continue
    data.append(d)

data = np.array(data)

print("Min wavefront time: ", np.min(data[:, 0] / data[:, 2]))
print("Max wavefront time: ", np.max(data[:, 0] / data[:, 2]))
print("Average wavefront time: ", np.median(data[:, 0] / data[:, 2]))

print("Min controls time: ", np.min(data[:, 1] / data[:, 2]))
print("Max controls time: ", np.max(data[:, 1] / data[:, 2]))
print("Average controls time: ", np.median(data[:, 1] / data[:, 2]))

print("Min total time: ", np.min(data[:, 0] + data[:, 1]))
print("Max total time: ", np.max(data[:, 0] + data[:, 1]))
print("Average total time: ", np.median(data[:, 0] + data[:, 1]))










