from matplotlib import pyplot as plt
import glob
import os
import json
from tqdm import tqdm
import numpy as np
from matplotlib.patches import Rectangle

data_folder = "/media/dhruv/a7519aee-b272-44ae-a117-1f1ea1796db6/2024/NAMO/bottleneck/ground_truth_dataset_center/data"

goal_positions = []
success_rates = []
for i in tqdm(os.listdir(data_folder)):
    if i == 100:
        break
    ctr = 0
    success_ctr = 0
    for j in os.listdir(os.path.join(data_folder, i)):
        info_file = os.path.join(data_folder, i, j, f"{j}_info.json")
        with open(info_file, "r") as f:
            info = json.load(f)
        if ctr == 0:
            goal_position = info["goal"][:2]
            goal_positions.append(goal_position)
        success_ctr += info["success"] * 1.0

        ctr += 1
    success_rates.append(success_ctr / ctr)

print(goal_positions)
print(success_rates)

goal_positions = np.array(goal_positions)


# plot the heatmap add rect patches of 0.1, 0.1 and the color corresponding to the success rate
plt.figure(figsize=(10, 10))
for i in range(len(goal_positions)):
    rect = Rectangle((goal_positions[i][0], goal_positions[i][1]), 0.1, 0.1, color=f"black", alpha=success_rates[i])
    plt.gca().add_patch(rect)
# plt.scatter(goal_positions[:, 0], goal_positions[:, 1], c=success_rates, cmap="Grey")
# plt.colorbar()
plt.axis("equal")
plt.show()