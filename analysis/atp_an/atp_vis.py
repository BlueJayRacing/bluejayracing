import matplotlib.pyplot as plt
import pandas as pd

# Step 1: Load the data
# Assuming the data file is named 'data.csv' and is in the same directory
data = pd.read_csv('0_28_rear_axle.txt', header=None, names=['time', 'value1', 'value2', 'value3'])

# Step 2: Plot the data
plt.figure(figsize=(10, 6))

# Plotting each value column as a separate line
plt.plot(data['time'], data['value1'], label='Value 1')
plt.plot(data['time'], data['value2'], label='Value 2')
plt.plot(data['time'], data['value3'], label='Value 3')

# Adding title and labels
plt.title('Data over Time')
plt.xlabel('Time')
plt.ylabel('Values')

# Adding legend
plt.legend()

# Display the plot
plt.show()
