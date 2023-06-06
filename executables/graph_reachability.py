import numpy as np 
import matplotlib.pyplot as plt
import yaml
import os
from matplotlib.patches import Rectangle
from xml.dom import minidom



fig, ax = plt.subplots()
fname = os.environ["DIRTMP_PATH"]+"out/ablation/wh_drrm100-c/visibility_data.txt"

num_nodes,p_visibility,arriveability,departability = np.loadtxt(fname, delimiter=",", unpack=True)


p_vis_plot, = plt.plot(num_nodes,p_visibility, label="p_visibility")
a_plot, = plt.plot(num_nodes,arriveability, label="arriveability")
d_plot, = plt.plot(num_nodes,departability, label="departability")

ax.legend(handles=[p_vis_plot, a_plot, d_plot])
plt.title('Warehouse DRRM: \"Visibility\" vs Reachability')

plt.xlabel('Number of Guards')

plt.savefig('wh-drrm100-vis.png')