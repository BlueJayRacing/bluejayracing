
import matplotlib.pyplot as plt
import numpy as np

from scipy.signal import butter, lfilter, freqz


def butter_lowpass(cutoff, fs, order=5):
  return butter(order, cutoff, fs=fs, btype='low', analog=False)

def butter_lowpass_filter(data, cutoff, fs, order=5):
  b, a = butter_lowpass(cutoff, fs, order=order)
  y = lfilter(b, a, data)
  return y


strain = []
load = []
time = []
e = []

signal_file = open("./failure_test.txt")
file_lines = signal_file.readlines()
for line in file_lines[100:]:
    values = line.split(',')
    time.append(float(values[0]))
    e.append(float(values[1]))
    strain.append(-2394203.854503341*(float(values[1])+-0.0003153185789580172))
    load.append(160029.09359836357*(float(values[2])+1.383420819423369e-05))


print(f"max gauge strain in V/V: {max(e)}")
print(f"min gauge strain in V/V: {min(e)}")

print(f"max gauged load in in-lbs: {max(strain)}")
print(f"max bucket load in in-lbs: {max(load)}")

qual_test_file = open("rear_axle_qual_test_012520024.csv", "w")
qual_test_file.write("time(us), Torque(in-lbs), Bucket Weight(lbs)\n")
for i in range(len(time)):
    qual_test_file.write(f'{time[i]},{strain[i]},{load[i]}\n')

fig, axs = plt.subplots(2)
fig.suptitle('Rear Axle Qual Test Data')
axs[0].set_title("Toruqe(in-lbs) vs Time(s)")
axs[1].set_title("Load Weight(lbs) vs Time(s)")


axs[0].scatter(time, butter_lowpass_filter(strain, 1, 32, 1), s=2)
axs[1].scatter(time, butter_lowpass_filter(load, 1, 32, 1), s=2)
plt.show()
