import matplotlib.pyplot as plt

# Sample data log provided as a string for demonstration
f = open("analog_ch.csv", "r")
f.readline()
data_log = f.read()

# Parse the data log
data = {}
for line in data_log.strip().split('\n'):
    parts = line.split(',')
    channel, value, timestamp = parts[0], float(parts[1]), float(parts[2])
    if channel not in data:
        data[channel] = []
    data[channel].append((timestamp, value))

# Plot the data in separate windows
for i, (channel, values) in enumerate(data.items()):
    timestamps, channel_values = zip(*values)  # Unpack the list of tuples
    plt.figure(i, figsize=(10, 4))
    plt.plot(timestamps, channel_values, label=channel)
    plt.xlabel('Time (s)')
    plt.ylabel('Value')
    plt.title(channel)
    plt.legend()

plt.show()