import numpy as np 
import matplotlib.pyplot as plt
import yaml
import os
from matplotlib.patches import Rectangle
from xml.dom import minidom



fig, ax = plt.subplots()
fname = os.environ["DIRTMP_PATH"]+"out/smrrm200/visibility_data.txt"

num_nodes,p_visibility,arriveability = np.loadtxt(fname, delimiter=",", unpack=True)


p_vis_plot, = plt.plot(num_nodes,p_visibility, label="p_visibility")
a_plot, = plt.plot(num_nodes,arriveability, label="reachability")

ax.legend(handles=[p_vis_plot, a_plot])
plt.title('\"Visibility\" vs Reachability')

plt.xlabel('Number of Guards')

plt.savefig('foo.png')