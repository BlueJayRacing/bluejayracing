import matplotlib.pyplot as plt

# Specify the file path
file_path = 'C:/Users/aaren/Desktop/bluejayracing/analysis/atp_an/0_28_rear_axle.txt'

# Read the file
with open(file_path, 'r') as file:
  data = file.readlines()

# Extract x and y values from the data
t = []
p = []
for line in data:
  values = line.split(',')
  if len(values) < 4:
    continue  
  t.append(float(values[0]))
  p.append(3000*(float(values[3])-.5)/4)

# Plot the data
plt.plot(t, p)
plt.xlabel('time')
plt.ylabel('pressure(psi)')
plt.title('Plot of 0_0_rear_axle.txt')
plt.show()