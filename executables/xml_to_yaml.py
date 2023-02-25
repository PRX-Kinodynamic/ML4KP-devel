import yaml
import os
import argparse
import numpy as np
from xml.dom import minidom
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

def euler2quat(euler):
    # Convert Euler angles to quaternion
    # https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    # Assumes that the quaternion is normalized
    roll, pitch, yaw = euler
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    w = cy * cr * cp + sy * sr * sp
    x = cy * sr * cp - sy * cr * sp
    y = cy * cr * sp + sy * sr * cp
    z = sy * cr * cp - cy * sr * sp
    return [x, y, z, w]

yaml_prefix = os.environ['DIRTMP_PATH'] + "resources/input_files/environments/"
xml_prefix  = os.environ['DIRTMP_PATH'] + "resources/models/mujoco/"

def xml_to_yaml_string(xml_fname):
    yaml_to_dump = {}
    yaml_to_dump["environment"] = {}
    yaml_to_dump["environment"]["type"] = "obstacle"
    yaml_to_dump["environment"]["geometries"] = []

    plt.figure(figsize=(8, 8))
    plt.xlim(-10,10)
    plt.ylim(-10,10)

    xmldoc = minidom.parse(xml_prefix + xml_fname)
    body_list = xmldoc.getElementsByTagName('body')
    for body in body_list:
        geom_list = body.getElementsByTagName('geom')
        for geom in geom_list:
            if geom.attributes['type'].value == "box":
                angle = 0
                if 'euler' in geom.attributes.keys():
                    angle = np.degrees([float(x) for x in geom.attributes['euler'].value.split()][2])
                box_center = [float(x) for x in geom.attributes['pos'].value.split()]
                box_dims   = [float(x) for x in geom.attributes['size'].value.split()]
                row = {}
                row["name"] = geom.attributes['name'].value
                row["collision_geometry"] = {}
                row["collision_geometry"]["type"] = "box"
                row["collision_geometry"]["material"] = "red"
                row["collision_geometry"]["dims"] = [2 * x for x in box_dims]
                row["config"] = {}
                row["config"]["position"] = box_center
                row["config"]["orientation"] = [float(x) for x in euler2quat([0,0,np.radians(angle)])]
                yaml_to_dump["environment"]["geometries"].append(row)
                # Plot it
                rect = Rectangle((box_center[0]-box_dims[0],box_center[1]-box_dims[1]),box_dims[0]*2,box_dims[1]*2,linewidth=1,\
                    edgecolor='r',facecolor='r',angle=angle, rotation_point='center')
                plt.gca().add_patch(rect)
    plt.show()
    # Dump it
    yaml_fname = yaml_prefix + xml_fname.split(".")[0] + ".yaml"
    with open(yaml_fname, 'w') as outfile:
        yaml.dump(yaml_to_dump, outfile, default_flow_style=False)
                

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("xml_fname", help="YAML file name",type=str,default="indoor.xml")
    args = parser.parse_args()
    xml_to_yaml_string(args.xml_fname)