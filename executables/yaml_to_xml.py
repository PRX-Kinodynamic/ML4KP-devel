import yaml
import os
import argparse

yaml_prefix = os.environ['DIRTMP_PATH'] + "resources/input_files/environments/"
xml_prefix  = os.environ['DIRTMP_PATH'] + "resources/models/mujoco/"

def yaml_to_xml_string(yaml_fname):
    with open(yaml_prefix + yaml_fname) as f:
        try:
            data = yaml.safe_load(f)
        except yaml.YAMLError as exc:
            print(exc)
    
    xml_string = ""
    obstacles = data["environment"]["geometries"]
    for obstacle in obstacles:
        if obstacle["collision_geometry"]["type"] == "box":
            box_center = obstacle["config"]["position"]
            box_dims   = obstacle["collision_geometry"]["dims"]
            xml_string += "<body name=\"obstacle_"+obstacle["name"]+"\">\n"
            xml_string += "\t<geom name=\"obstacle_"+obstacle["name"]+"\" type=\"box\" pos=\""+str(box_center[0])+" "+str(box_center[1])+" "+str(box_center[2])+"\" size=\""+str(box_dims[0]*0.5)+" "+str(box_dims[1]*0.5)+" "+str(box_dims[2]*0.5)+"\" rgba=\"1.0 0.0 0.0 1\"/>\n"
            xml_string += "</body>\n"
        else:
            print("Only box obstacles are supported")
            return ""
    return xml_string

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("yaml_fname", help="YAML file name")
    args = parser.parse_args()
    xml_string = "<mujoco model = \"obstacles\">\n<worldbody>\n"
    xml_string += yaml_to_xml_string(args.yaml_fname)
    xml_string += "</worldbody></mujoco>"
    xml_fname = xml_prefix + args.yaml_fname.replace("yaml", "xml")
    with open(xml_fname, "w+") as f:
        f.write(xml_string)
    
