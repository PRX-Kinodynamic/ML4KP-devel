import os
import xml.etree.ElementTree as ET
from glob import glob
import numpy as np
from matplotlib import pyplot as plt
from tqdm import tqdm
from matplotlib.patches import Rectangle, Circle
with open('/home/dhruv/2024/projects/ml4kp_ktamp/resources/models/push/model_bottlenecks.xml', 'r') as file:
    mjx_content = file.read()
root = ET.fromstring(mjx_content)
VERTS = '/home/dhruv/2024/projects/ml4kp_ktamp/build/output_v.txt'
EDGES = '/home/dhruv/2024/projects/ml4kp_ktamp/build/output_e.txt'
PATH_EDGES = '/home/dhruv/2024/projects/ml4kp_ktamp/build/output_path.txt'


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

verts = []
with open(VERTS, 'r') as f:
    for line in tqdm(f):
        x = line.strip().split(" ")
        verts.append(x)
        ax.scatter(float(x[0]), float(x[1]), color='black')

# with open(EDGES, 'r') as f:
#     for line in tqdm(f):
#         x, y = line.strip().split(" ")
#         ax.plot([float(verts[int(x)][0]), float(verts[int(y)][0])], [float(verts[int(x)][1]), float(verts[int(y)][1])], color='black')

with open(PATH_EDGES, 'r') as f:
    for line in tqdm(f):
        x, y = line.strip().split(" ")
        ax.scatter(float(verts[int(x)][0]), float(verts[int(x)][1]), color='red', s=100)
        ax.plot([float(verts[int(x)][0]), float(verts[int(y)][0])], [float(verts[int(x)][1]), float(verts[int(y)][1])], color='red')

ax.scatter(0, 0, s=100, color='green')
ax.scatter(1.5, 0.0, s=100, color='blue')

plt.show()

