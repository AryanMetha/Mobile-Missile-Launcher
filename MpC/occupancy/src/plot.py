import matplotlib.pyplot as plt
import numpy as np

import csv
xPoints = np.array([])
yPoints = np.array([])

with open('points.csv', mode ='r') as file:    
       csvFile = csv.DictReader(file)
       for lines in csvFile:
            xPoints = np.append(xPoints, float(lines["x"]))
            yPoints = np.append(yPoints, float(lines["y"]))
            print(lines)

startX = xPoints[0]
startY = yPoints[0]
""" xPoints = xPoints[1:]
yPoints = yPoints[1:] """

xPoints = np.sort(xPoints ,  axis = None)
yPoints = np.sort(yPoints,  axis = None)
MapEdge = np.array([0,50])
plt.plot(xPoints, yPoints, 'o:r')
plt.plot(startX, startY, 'o:g')

plt.plot(MapEdge, MapEdge, 'o')
plt.title("Path Plan")
plt.xlabel("X")
plt.ylabel("Y")
plt.show()
