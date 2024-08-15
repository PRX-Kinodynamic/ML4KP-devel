def process_subgoals(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    output = []
    target_counter = 1

    for line in lines:
        if line.strip() == "0 0 0.05":  # Skip the initial line
            continue
        x, y, z = line.split()
        site_name = f"target{target_counter}"
        pos = f"{x} {y} 0.0"
        size = f"{z}"
        rgba = "0 1 0 0.1"
        site_output = f'<site name="{site_name}" pos="{pos}" size="{size}" rgba="{rgba}" />'
        output.append(site_output)
        target_counter += 1
    
    # Add the final site
    site_output = f'<site name="target{target_counter}" pos="1.7 0.0 0.0" size="0.05" rgba="0 1 0 0.1" />'
    output.append(site_output)
    
    for line in output:
        print(line)

# Call the function with the path to your subgoals.txt file
process_subgoals('/home/dhruv/2024/projects/ml4kp_ktamp/build/turning_interactions/12/subgoals_room2room_finer.txt')