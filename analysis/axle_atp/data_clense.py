import matplotlib.pyplot as plt
import re
import numpy as np
import math
import json


def create_new_data_file(file_name, data):
    fileObj = open(file_name, "w")
    fileObj.writelines(data)
    fileObj.close()
    

file = "axle5_second_half.txt"

# put the manual time values in here
times = []

timeIndex = 0

# manual torque data should be placed here
inputData = [[5],[6],[7],[8]]

# torque data from csv will be placed here
torqueData = []

# dictionary with a list of dictionaries
dataDict = {}

# fill with all the times from the csv
timeValues = [("1153327988", "1153798694")]

# just to minimize the amount of data
count = 0

inRange = False

data_file = open(file)

file_lines = data_file.readlines()


for line in file_lines:
    values = line.split(',')
    
    if round(float(timeValues[timeIndex][0]), 4) == round(float(values[0]), 4):
        inRange = True
    
    if inRange:
        torqueData.append(float(values[2]))
        
    if round(float(timeValues[timeIndex][1]), 4) == round(float(values[0]), 4):
        inRange = False
        # NOTE: the times are the stop times
        dataDict[f'{timeValues[timeIndex][0]}-{timeValues[timeIndex][1]}'] = {"torque known": inputData[timeIndex], "torque measured": torqueData}
        timeIndex += 1
        torqueData = []
        if timeIndex == len(timeValues):
            break
    
    times.append(float(values[0]))
    


json_object = json.dumps(dataDict, indent=2)
with open("torqueData.json", "w") as outfile:
    outfile.write(json_object)


