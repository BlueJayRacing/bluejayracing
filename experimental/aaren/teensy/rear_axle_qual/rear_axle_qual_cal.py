import matplotlib.pyplot as plt
# import re
import numpy as np
# import math
from scipy.signal import butter, lfilter, freqz


def butter_lowpass(cutoff, fs, order=5):
  return butter(order, cutoff, fs=fs, btype='low', analog=False)

def butter_lowpass_filter(data, cutoff, fs, order=5):
  b, a = butter_lowpass(cutoff, fs, order=order)
  y = lfilter(b, a, data)
  return y

file_path = "./"
names = ["static_zero.txt", "static_2664inlbf_strain_2.9deg.txt", "strain_30.93lbs_load.txt"]
cals = [0, 2664, 30.93]
strain_res = []
load_res = []

for name in names:
  strain = []
  load = []
  time = []

  signal_file = open(file_path + name)
  file_lines = signal_file.readlines()
  for line in file_lines:
    values = line.split(',')
    time.append(float(values[0]))
    strain.append(float(values[1]))
    load.append(float(values[2]))
  strain_res.append(np.mean(strain))
  load_res.append(np.mean(load))


print(f'Strain scale:{cals[1]/(strain_res[1] - strain_res[0])}\t offset:{-1*strain_res[0]}')
print(f'Load scale:{cals[2]/(load_res[2] - load_res[0])}\t offset:{-1*load_res[0]}')



fig, axs = plt.subplots(2)
fig.suptitle('Rear Axle Qual Test Calibration')
axs[0].set_title("Strain Gauge Calibration")
axs[1].set_title("Load Cell Calibration")
# axs.set_ylabel("Torque(in-lbs)")
# axs.set_xlabel("Time since logging(s)")



axs[0].scatter(cals, strain_res, s=5)
axs[1].scatter(cals, load_res, s=5)
plt.show()