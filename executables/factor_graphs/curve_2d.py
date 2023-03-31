import math
import numpy as np

def spiral2d(t):
	return t-math.sin(30*t), 1-math.cos(30*t);

def straight(t):
	return t, 1

def side(t):
	return 1, t

time_step=0.01
curve_name="spiral"
duration=1
file = open(curve_name + "_" + str(time_step) + ".txt", 'w')


for t in np.arange(0, duration, time_step):
	x,y=spiral2d(t);
	# x,y=straight(t);
	# x,y=side(t);
	file.write(str(t) + " " + str(x) + " " + str(y) + "\n")

file.flush()