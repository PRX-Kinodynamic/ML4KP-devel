import yaml

def convert_yaml_to_dict(path):
    """
    Load a YAML file and convert it to a dictionary.
    
    Args:
        path (str): Path to the YAML file
        
    Returns:
        dict: The loaded configuration
    """
    with open(path, 'r') as f:
        config = yaml.safe_load(f)
    return config 