#importing necessary libraries
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt

directory = "/home/r1/"

f = open(directory+"test.csv", "r")
csv = f.read()

x=[]
y=[]
z=[]

csv = csv.split("\n")
csv.pop()
for l in csv:
    point = l.split(";")
    print (point)
    x.append(float(point[0]))
    y.append(float(point[1]))
    z.append(float(point[2]))



f.close()

fig = plt.figure('Parametric curve')
ax  = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z, '-r', linewidth = 3)


#setting the labels
ax.set_xlabel('X', fontsize = 12)
ax.set_ylabel('Y', fontsize = 12)
ax.set_zlabel('Z', fontsize = 12)

plt.title('Parametric Curve', fontsize = 14)
plt.show()