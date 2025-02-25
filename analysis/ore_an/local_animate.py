import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R

# Modified function to load data from file, including quaternions
def load_data_with_quaternion(filename):
    points_data = []
    quaternions_data = []
    with open(filename, "r") as file:
        for line in file.readlines():
            parts = line.strip().split(',')
            points_data.append([float(part) for part in parts[:3]])  # ENU coordinates
            quaternions_data.append([float(part) for part in parts[3:7]])  # Quaternion data
    return np.array(points_data), np.array(quaternions_data)

# Load points and quaternions from file
points, quaternions = load_data_with_quaternion("sam_lihn.txt")

# Set up the figure and axes for 3D plotting
fig = plt.figure(figsize=(14, 7))

# Plot for the ENU coordinates
ax1 = fig.add_subplot(121, projection='3d')
ax1.set_title('ENU Coordinates')

# Plot for the quaternion vector visualization
ax2 = fig.add_subplot(122, projection='3d')
ax2.set_title('Quaternion Vector')

# Initial setup for the plot elements in ax1
line, = ax1.plot([], [], [], 'r-', marker='x', markersize=5)
point, = ax1.plot([], [], [], 'ro')

# Initial setup for the quaternion vector visualization in ax2
quaternion_vector, = ax2.plot([], [], [], 'b-', linewidth=2)

# Setting the axes limits based on the data range plus some padding
padding = 10
ax1.set_xlim([points[:,0].min() - padding, points[:,0].max() + padding])
ax1.set_ylim([points[:,1].min() - padding, points[:,1].max() + padding])
ax1.set_zlim([points[:,2].min() - padding, points[:,2].max() + padding])
ax2.set_xlim([-1, 1])
ax2.set_ylim([-1, 1])
ax2.set_zlim([-1, 1])

# Update function for the animation
def update(num, points, quaternions, line, point, quaternion_vector):
    # Update ENU coordinates plot
    end = max(0, num-100)  # Keep last 100 points
    line.set_data(points[end:num, 0], points[end:num, 1])
    line.set_3d_properties(points[end:num, 2])
    point.set_data(points[num, 0], points[num, 1])
    point.set_3d_properties(points[num, 2])

    # Update quaternion vector visualization
    quaternion = R.from_quat(quaternions[num])
    vector = quaternion.apply([1, 0, 0])  # Apply quaternion to a unit vector
    quaternion_vector.set_data([0, vector[0]], [0, vector[1]])
    quaternion_vector.set_3d_properties([0, vector[2]])

    return line, point, quaternion_vector

# Create the animation
ani = FuncAnimation(fig, update, frames=range(0,len(points),1000), fargs=(points, quaternions, line, point, quaternion_vector), interval=0, blit=False)

plt.show()
