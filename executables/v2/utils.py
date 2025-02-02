import yaml

def convert_yaml_to_dict(yaml_file):
    with open(yaml_file, 'r') as file:
        return yaml.safe_load(file)
    