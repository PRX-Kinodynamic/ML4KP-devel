import matplotlib.pyplot as plt
import numpy as np

from matplotlib.colors import Normalize
from matplotlib.markers import MarkerStyle
from matplotlib.text import TextPath
from matplotlib.transforms import Affine2D



upper_bound = np.pi/2
lower_bound = 0

with open("filename.txt") as f:
    din = np.genfromtxt("filename.txt", delimiter=",")
    lipshitz = din[-1][-1]

    positions = din[:, [0, 1]]  
    angles = din[:, 2]
    img_err = din[:, 10]
    atten = din[:, 11]/lipshitz
    data = zip(angles, values, positions)

    cmap = plt.get_cmap("viridis_r")
    fig, ax = plt.subplots()
    fig.suptitle("", size=14)
    for angle, value, pos in data:
        if(angle < upper_bound and angle > lower_bound):
            t = Affine2D().rotate(angle)
            m = MarkerStyle(8, transform=t)
            if(value == 99/lipshitz):
                ax.plot(pos[0], pos[1], marker=m, color="red")
            else:
                ax.plot(pos[0], pos[1], marker=m, color=cmap(value))
    fig.colorbar(plt.cm.ScalarMappable(norm=Normalize(0, lipshitz), cmap=cmap),
                ax=ax, label="Lipshitz param")
    ax.set_xlabel("X position [m]")
    ax.set_ylabel("Y position [m]")

    plt.savefig("filename.png")