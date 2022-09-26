import numpy as np 
import matplotlib.pyplot as plt 
import yaml 
import os
import xml.etree.ElementTree as ET
from matplotlib.patches import Circle

world_fname = "/Users/aravind/Downloads/BARN_Dataset/world_files/world_299.world"


yaml_to_dump = {}
yaml_to_dump["environment"] = {}
yaml_to_dump["environment"]["type"] = "obstacle"
yaml_to_dump["environment"]["geometries"] = []

tree = ET.parse(world_fname)
plt.figure(figsize=(4,8))
for item in tree.findall('./world/model'):
    # If item's name starts with "unit_cylinder"
    if item.attrib['name'].startswith("unit_cylinder"):
        row = {}
        row["name"] = item.attrib['name']
        row["collision_geometry"] = {}
        row["collision_geometry"]["type"] = "cylinder"
        row["collision_geometry"]["material"] = "red"
        # Print its pose
        center = item.find('pose').text.split(" ")
        center = [float(x) for x in center]
        print(center)
        # Find its radius and length
        radius = item.find('link/collision/geometry/cylinder/radius').text
        r = item.find('link/collision/geometry/cylinder/radius').text
        h = item.find('link/collision/geometry/cylinder/length').text
        row["collision_geometry"]["radius"] = float(radius)
        row["collision_geometry"]["height"] = float(h)
        row["config"] = {}
        row["config"]["position"] = [center[0],center[1],float(h)/2]
        row["config"]["orientation"] = [0,0,0,1]
        yaml_to_dump["environment"]["geometries"].append(row)
        # Plot it
        circle = Circle((float(center[0]),float(center[1])),float(radius),linewidth=1,edgecolor='r',facecolor='r')
        plt.gca().add_patch(circle)

plt.xlim(-5,0)
plt.ylim(0,10)
plt.show()

with open(os.environ["DIRTMP_PATH"]+"resources/input_files/environments/BARN.yaml",'w') as f:
    yaml.dump(yaml_to_dump,f)