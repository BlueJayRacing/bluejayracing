import matplotlib.pyplot as plt
import numpy as np

# Read data from the file
with open("analog_ch.csv", "r") as file:
    file.readline()  # Skip header
    data_log = file.readlines()

# Parse the data log
data = {}
for line in data_log:
    parts = line.strip().split(',')
    channel, value, timestamp = parts[0], float(parts[1]), float(parts[2])
    if channel not in data:
        data[channel] = []
    data[channel].append((round(timestamp, 2), value))  # Round timestamps for better alignment

# Function to align data by timestamps with a tolerance
def align_data(data1, data2):
    timestamps1, values1 = zip(*data1)
    timestamps2, values2 = zip(*data2)
    common_times = set(timestamps1).intersection(timestamps2)
    aligned_data1 = [value for time, value in data1 if time in common_times]
    aligned_data2 = [value for time, value in data2 if time in common_times]
    sorted_times = sorted(list(common_times))
    return sorted_times, aligned_data1, aligned_data2

# Attempt to align data
timestamps, front_right, rear_right = align_data(data["SHOCK_LEN_FRONT_RIGHT"], data["SHOCK_LEN_REAR_RIGHT"])
differences = np.array(front_right) - np.array(rear_right)
derivatives = np.diff(differences) / np.diff(timestamps)  # Calculate derivatives

differences /= 2300  # Convert to meters
derivatives /= 10000


# Check for sufficient data points
if derivatives.size > 0:
    window_size = 10  # Define the window size
    if derivatives.size >= window_size:
        # Calculate moving standard deviation
        windowed_std_dev = np.convolve(np.abs(derivatives), np.ones(window_size) / window_size, mode='valid')
        timestamps_std_dev = timestamps[window_size-1:]  # Adjust timestamps for plotting moving std deviation
        
        windowed_std_dev = np.sqrt(windowed_std_dev)  # Calculate standard deviation


        # Plotting
        plt.figure(figsize=(12, 8))
        plt.subplot(311)
        plt.plot(timestamps, differences, label='Difference')
        plt.title('Difference Between Front Right and Rear Right Shock Lengths')
        plt.xlabel('Time (s)')
        plt.ylabel('Pitch Height Difference (in)')
        plt.legend()

        plt.subplot(312)
        plt.plot(timestamps[1:], derivatives, label='Derivative')
        plt.title('Derivative of Differences')
        plt.xlabel('Time (s)')
        plt.ylabel('Pitch Velocity (in/s)')
        plt.legend()

        plt.subplot(313)
        plt.plot(timestamps_std_dev[:-1], windowed_std_dev, label='Moving Std Dev')
        plt.title('Moving Standard Deviation of the Derivative')
        plt.xlabel('Time (s)')
        plt.ylabel('Standard Deviation')
        plt.legend()

        plt.tight_layout()
        plt.show()
    else:
        print("Not enough data points after derivative to calculate moving standard deviation.")
else:
    print("No derivatives could be calculated due to insufficient common data points.")
