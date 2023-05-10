#importing necessary libraries
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt


directory = "/home/r1/"
nb_point_wanted = 100

#Parametric equations in cartesian coordinates
r   = .05 #ray
t = np.linspace(0, 2* np.pi, 100) #timeline/period

x_offset = 0.0
y_offset = 0.05
z_offset = 0.15

x = 2*r*np.sqrt(2)*((np.sin(t))/(1+(np.cos(t)**2))) + x_offset
y = 2*r*np.sqrt(2)*((np.sin(t)*np.cos(t))/(1+(np.cos(t)**2))) + y_offset
z = 2*r*np.sqrt(2)*np.cos(t) + z_offset


fig = plt.figure('Parametric curve')
ax  = fig.add_subplot(111, projection='3d')
ax.plot(x, y, z, '-r', linewidth = 3)

coords = []
for i in range(len(t)):
    coords.append([x[i],y[i],z[i]])

for i in range(len(coords)):
    if i == 0:
        coords[i].append(0)
    else:
        coords[i].append(coords[i-1][3]+np.sqrt(((coords[i-1][0]-coords[i][0])**2)+((coords[i-1][1]-coords[i][1])**2)+((coords[i-1][2]-coords[i][2])**2)))
for i in coords:
    print(i)

total_distance = coords[-1][-1]
print("total distance")
print(total_distance)

final_points = []
for p in coords:
    if p[-1] >= total_distance/(nb_point_wanted-len(final_points)):
        final_points.append([p[0],p[1],p[2]])

print("___")
print("___")
print("___")
for p in final_points:
    print(p)
    
csvFormat = ""
for point in final_points:
    csvFormat = csvFormat + str(point[0])+";" + str(point[1])+";" + str(point[2])+"\n"
print(csvFormat)

f = open(directory+"test.csv", "w")
f.write(csvFormat)
f.close()