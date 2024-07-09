import os
import xml.etree.ElementTree as ET
from glob import glob
import numpy as np
from matplotlib import pyplot as plt
from tqdm import tqdm
from matplotlib.patches import Rectangle, Circle

with open('/home/dhruv/2024/projects/ml4kp_ktamp/resources/models/push/model_hard1.xml', 'r') as file:
    mjx_content = file.read()
FOLDER = f'/home/dhruv/2024/projects/NAMO/out/only_navigate/trees/*'
root = ET.fromstring(mjx_content)


fig, ax = plt.subplots()

for body in root.findall('.//body'):
    pos = body.get('pos').split()
    pos = [float(p) for p in pos]
    
    for geom in body.findall('.//geom'):
        geom_type = geom.get('type')
        geom_name = geom.get('name')
        if 'cube' in geom_name or 'wall' in geom_name: 
            size = geom.get('size').split()
            size = [float(s) for s in size]
            if geom_type == 'box':
                # Box size is half-length in each dimension
                w, h = size[0]*2, size[1]*2
                rect = Rectangle((pos[0] - w/2, pos[1] - h/2), w, h, edgecolor='r', facecolor='none')
                ax.add_patch(rect)

colors = ['blue', 'green', 'red', 'purple']
for task_id, folder in enumerate(sorted(list(glob(FOLDER)))):
    points = []
    for i in tqdm(list(glob(folder+'/*.txt'))):
        with open(i, 'r') as f:
            for line in f:
                points.append(np.array(eval(line.split(': ')[1].strip()))[:2].tolist())
                # y.append(float(line.split()[1]))
        points = np.array(points)
        ax.plot(points[:, 0], points[:, 1], color=colors[task_id])
        points = []
    
ax.set_xlim(-0.1, 1.5)
ax.set_ylim(-0.25, 0.25)
# ax.scatter(0.0, 0, color='blue', label='Start')
# ax.scatter(0.5, 0, color='green', label='Goal')
plt.legend()
plt.show()
        
