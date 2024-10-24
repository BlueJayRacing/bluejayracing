import matplotlib.pyplot as plt
import re
import numpy as np
import math


tau = []
time = []

for i in range(16):
    num = ""

    if (i > 9):
      num = "0" + str(i)
    else:
       num = "00" + str(i)

    torq_file = open("../../OREGON/rollover1/ORE_strain_0_"+num)
    file_lines = torq_file.readlines()

    for line in file_lines:
      values = line.split(',')
      time.append(float(values[0]))
      tau.append(float(values[1])*7.05-246)

break_time = time[-1]

for i in range(40):
    num = ""

    if (i > 9):
      num = "0" + str(i)
    else:
       num = "00" + str(i)

    torq_file = open("../../OREGON/ORE_strain_0_"+num)
    file_lines = torq_file.readlines()

    for line in file_lines:
      values = line.split(',')
      time.append(float(values[0])+break_time)
      tau.append(float(values[1])*7.05-246)


start = 3150000
dead = 5700000
istart = 1095000+start
iend = 1790000+start
tau = tau[start:istart] + tau[iend:dead]
time = time[start:istart] + time[iend:dead]

# f = open("gps_vis.txt", "w")
# for i in range(len(gps_lat)):
#   s = gps_lat[i] + ',' + gps_lon[i]+','+gps_alt[i] + '\n'
#   f.writelines([s])
# f.close()
print(max(tau))
print(min(tau))
print(np.average(tau))

print(time[-1]/len(time))

#time = np.arange(0, len(tau))


fig, axs = plt.subplots(1)
fig.suptitle('Oregon Endurance')
axs.set_title("Torque vs Time")
axs.set_ylabel("Torque(in-lbs)")
axs.set_xlabel("Time since logging(s)")


axs.scatter(time[0:-1:50], tau[0:-1:50], s=1)

plt.show()


