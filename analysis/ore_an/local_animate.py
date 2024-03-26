import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

# Function to load data from file
def load_data(filename):
    data = []
    with open(filename, "r") as file:
        for line in file.readlines():
            parts = line.strip().split(',')
            # Convert string to float and append to data list
            data.append([float(part) for part in parts[:3]])  # Assuming only ENU coordinates are needed
    return np.array(data)

# Load points from file
points = load_data("sam_lihn.txt")

# Set up the figure and axis for 3D plotting
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Initial setup for the plot elements we intend to update
line, = ax.plot([], [], [], 'r-', marker='x', markersize=5)  # For the trace
point, = ax.plot([], [], [], 'ro')  # For the leading point

# Setting the axes limits based on the data range plus some padding
padding = 10

xmi = points[:,0].min() - padding
xma = points[:,0].max() + padding
ymi = points[:,1].min() - padding
yma = points[:,1].max() + padding
zmi = points[:,2].min() - padding
zma = points[:,2].max() + padding

# Animation update function
def update(num, points, line, point):
    end = max(0, num-500)  # Keep last 100 points
    line.set_data(points[end:num, 0], points[end:num, 1])
    line.set_3d_properties(points[end:num, 2])
    point.set_data(points[num, 0], points[num, 1])
    point.set_3d_properties(points[num, 2])
    plt.cla() # Clear the current axis
    ax.set_xlim([xmi, xma])
    ax.set_ylim([ymi, yma])
    ax.set_zlim([zmi, zma])
    ax.plot(points[end:num, 0], points[end:num, 1], points[end:num, 2], marker='x')
    ax.scatter(*points[num], color='red')  # Highlight the current point in red

    
    return line, point

# Create animation
ani = FuncAnimation(plt.gcf(), update, frames=range(0, len(points),10), fargs=(points, line, point), interval=0, blit=False, repeat=False)

plt.show()  