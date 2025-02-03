import matplotlib.pyplot as plt
import numpy as np

import math
import csv
xPoints = np.array([])
yPoints = np.array([])
z = 0.3
conversionUnit = 1

with open('points.csv', mode ='r') as file:    
       csvFile = csv.DictReader(file)
       for lines in csvFile:
            xPoints = np.append(xPoints, float(lines["x"]))
            yPoints = np.append(yPoints, float(lines["y"]))
            print(lines)

thetha = math.atan((yPoints[1]-yPoints[0])/(xPoints[1]-xPoints[0]))
x = 2*z*conversionUnit*math.cos(thetha) + xPoints[-1]
y = 2*z*conversionUnit*math.sin(thetha) + yPoints[-1]

targetX = np.array([x])
targetY = np.array([y])

startX = xPoints[0]
startY = yPoints[0]

endX = xPoints[-1]
endY = yPoints[-1]


MapEdge = np.array([0,50])
plt.plot(xPoints, yPoints, 'o:r')
plt.plot(startX, startY, 'o:m')
plt.plot(endX, endY, 'o:g')
plt.plot(targetX, targetY, 'o:k')


plt.plot(MapEdge, MapEdge, 'o')
plt.title("Path Plan")
plt.xlabel("X")
plt.ylabel("Y")
plt.show()
