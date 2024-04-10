import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Function to read data from the file
def read_data_from_file(filename):
    local = []
    with open(filename, "r") as f:
        for line in f.readlines():
            parts = line.strip().split(',')
            if len(parts) == 7:  # Ensure correct data format
                local.append(tuple(map(float, parts)))
    return local

# Reading data
local = read_data_from_file("sam_lihn.txt")

# Extracting ENU coordinates for plotting
points = np.array([item[:3] for item in local])

end = len(points) - 7000

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(points[:end, 0], points[:end, 1], points[:end, 2], marker='x')
ax.scatter(*points[0], color='red')  # Highlight the first point in red

plt.show()


