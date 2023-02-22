import math
import numpy as np

def spiral2d(t):
	return t-math.sin(10*t), 1-math.cos(10*t);

file = open("curve_0p001.txt", 'w')

time_step=0.001
duration=1


for t in np.arange(0, duration, time_step):
	x,y=spiral2d(t);
	file.write(str(t) + " " + str(x) + " " + str(y) + "\n")

file.flush()