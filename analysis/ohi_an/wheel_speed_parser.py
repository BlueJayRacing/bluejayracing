import matplotlib.pyplot as plt
import re
import numpy as np
import math

time = []
cds_s = []

for i in range(0,62):
    if (i%10 == 0):
      print(str(round(i/.62,2)) + "% parsed")
    num = ""

    if (i > 9):
      num = "0" + str(i)
    else:
       num = "00" + str(i)

    imu_file = open("../../OHIO/endurance/OHI_center_0_"+num)
    file_lines = imu_file.readlines()

    for line in file_lines:
      values = line.split(',')
      time.append(float(values[0]))
      cds_s.append(float(values[1].strip('\n')))

cds_f = []
window_size = 64
SAMPLING_RATE_HZ = 860


for i in range(len(cds_s)-window_size):
  if (i%(len(cds_s)/20) == 0):
      print(str(round(100*i/len(cds_s),2)) + "% processed")

  sp = np.abs(np.fft.fft(cds_s[i:i + window_size], 1024))
  dom_freq = np.argmax(sp[1: len(sp)//2])
  if abs(np.max(sp[1: len(sp)//2])) > 400:
    cds_f.append(dom_freq)
  else:
    cds_f.append(0)

mph =  np.array(cds_f) * 60 * 0.0397170837 * (SAMPLING_RATE_HZ) / (1024 * 24)

f = open("wheel_freq.txt", "w")
for i in range(0, len(cds_f), 1):
  s = str(time[i+window_size]) + "," + str(cds_f[i]) + "," + str(mph[i]) + '\n'
  f.writelines([s])
f.close()


fig, axs = plt.subplots(3)
fig.suptitle('S, F, mph')
axs[0].set_title("raw signal")
axs[1].set_title("dominant forier transform")
axs[2].set_title("mph")


print(len(cds_s))
print(len(time))
print(len(time[0:len(time)-window_size]))

print(max(mph))


axs[0].scatter(time, cds_s, s=1)
axs[1].scatter(time[0:len(time)-window_size], cds_f, s=1)
axs[2].scatter(time[0:len(time)-window_size], mph, s=1)

plt.show()


