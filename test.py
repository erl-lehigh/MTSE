import matplotlib.pyplot as plt
import shapely.geometry as sp
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import cos, sin, pi, sqrt
from shapely.geometry import Point
from shapely.geometry import LineString

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

# Variables for Pure Pursuit algorithm

u = (0, 0) # initial velocity
#acc =

point = Point(2,3)  # vehicle coords

lookahead = 4  # look ahead distance set to 4 units

line = LineString([(1, 1), (8, 4)])

x,y = line.xy
ax1.plot(x,y)

ax1.set_ylim(ymin=0, ymax=10)
ax1.set_xlim(xmin=0, xmax=10)

ax1.arrow(point.x,point.y,dx=1,dy=1)
ax1.plot(point.x,point.y,'ob')

d = line.project(point)
i = line.interpolate(d)
ax1.plot(i.x, i.y, 'og')
ax1.axis('equal')

project = LineString([(point.x, point.y), (i.x, i.y)])
x1,y1 = project.xy
ax1.plot(x1,y1)

close_path = project.length
dist_on_path = sqrt((lookahead ** 2) - (close_path ** 2))

t = dist_on_path/line.length


future_pt = Point((1-t)*i.x + t* line.coords[1][0], (1-t)*i.y + t*line.coords[1][1])
#future_pt = line.ar

ax1.plot(future_pt.x, future_pt.y, 'oy')


goal = LineString([(point.x, point.y), (future_pt.x, future_pt.y)])
x2,y2 = goal.xy
ax1.plot(x2,y2, color='y')


print('vehicle coords: '+ str(point.x) + ',' + str(point.y) + '\n')
print('closest pt: ' + str(i.x) + ',' + str(i.y) + '\n')
print('path length: ' + str(line.length) + '\n')
print('closest length: ' + str(project.length))
print('dist on path length: ' + str(dist_on_path))
#print(type(line.coords[1][1]))

plt.show()

