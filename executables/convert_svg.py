import numpy as np
import matplotlib.pyplot as plt
import os
import yaml
from matplotlib.patches import Rectangle

fname = "/Users/aravind/Downloads/warehouse_boxes.txt"
boxes = np.loadtxt(fname,delimiter=" ")
print(boxes.shape)

# Each line in boxes.txt is of the form (x_min, x_max, y_min, y_max)
x_min = y_min = np.inf 
x_max = y_max = -np.inf

for box in boxes:
    x_min = min(x_min,box[0])
    x_max = max(x_max,box[1])
    y_min = min(y_min,box[2])
    y_max = max(y_max,box[3])

converted_x_min = -0.5 * (np.ceil(x_max - x_min))
converted_y_min = -0.5 * (np.ceil(y_max - y_min))
converted_x_max = 0.5 * (np.ceil(x_max - x_min))
converted_y_max = 0.5 * (np.ceil(y_max - y_min))

# Transform the boxes to the new coordinate system
converted_boxes = []
for box in boxes:
    converted_boxes.append([box[0]-x_min+converted_x_min,box[1]-x_min+converted_x_min,
    box[2]-y_min+converted_y_min,box[3]-y_min+converted_y_min])
converted_boxes = np.vstack(converted_boxes)



# Plot the boxes
plt.figure(figsize=(8,8))

rect = Rectangle((-5,-4),0.52,0.5,linewidth=1,edgecolor='g',facecolor='g')
plt.gca().add_patch(rect)

for box in converted_boxes:
    rect = Rectangle((box[0],box[2]),box[1]-box[0],box[3]-box[2],linewidth=1,edgecolor='r',facecolor='r')
    plt.gca().add_patch(rect)
# Plot the new coordinate system
plt.xlim(converted_x_min,converted_x_max)
plt.ylim(converted_y_min,converted_y_max)
plt.show()

yaml_to_dump = {}
yaml_to_dump["environment"] = {}
yaml_to_dump["environment"]["type"] = "obstacle"
yaml_to_dump["environment"]["geometries"] = []

box_num = 0
for box in converted_boxes:
    row = {}
    row["name"] = "box_{}".format(box_num)
    row["collision_geometry"] = {}
    row["collision_geometry"]["type"] = "box"
    row["collision_geometry"]["material"] = "red"
    row["collision_geometry"]["dims"] = [float(np.round(box[1]-box[0],2)),float(np.round(box[3]-box[2],2)),0.2]
    row["config"] = {}
    row["config"]["position"] = [float(np.round(box[0]+0.5*(box[1]-box[0]),2)),float(np.round(box[2]+0.5*(box[3]-box[2]),2)),0.1]
    row["config"]["orientation"] = [0,0,0,1]
    yaml_to_dump["environment"]["geometries"].append(row)
    box_num += 1

with open(os.environ["DIRTMP_PATH"]+"resources/input_files/environments/warehouse_benchmr.yaml","w") as f:
    yaml.dump(yaml_to_dump, f, default_flow_style=None)