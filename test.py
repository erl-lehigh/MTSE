import matplotlib.pyplot as plt
import shapely.geometry as sp
import numpy as np


import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import cos, sin, pi
from shapely.geometry import Point
from shapely.geometry import LineString


fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)


# Variables for Pure Pursuit algorithm

#A = Point(1,1)
#B = Point(3,2)
#AB = LineString([A, B])

velocity = (0, 0)


point = Point(2,3)


line = LineString([(1, 1), (8, 4)])

x,y = line.xy
ax1.plot(x,y)

ax1.set_ylim(ymin=0, ymax=10)
ax1.set_xlim(xmin=0, xmax=10)

ax1.arrow(point.x,point.y,dx=1,dy=1)
ax1.plot(point.x,point.y,'ob')

#d = line.project(point)
##i = line.interpolate(d)
#ax1.plot(i.x, i.y, 'og')



x = np.array(point.coords[0])

u = np.array(line.coords[0])
v = np.array(line.coords[len(line.coords)-1])

n = v - u
n /= np.linalg.norm(n, 2)

P = u + n*np.dot(x - u, n)

ax1.plot(P.x, P.y, 'og')


'''

x = 1;                          # X coordinate of arrow start
y = 2;                          # Y coordinate of arrow start
theta = pi/4;                   # Angle of arrow, from x-axis
L = 2;                         # Length of arrow
xEnd = x+L*cos(theta);          #X coordinate of arrow end
yEnd = y+L*sin(theta);         # Y coordinate of arrow end
points = np.linspace(0, theta);    # 100 points from 0 to theta
xCurve = x+(L/2).* cos(points);  # X coordinates of curve
yCurve = y+(L/2).*sin(points);  # Y coordinates of curve
ax1.plot(x+[-L L], [y y], '--k');   # Plot dashed line
plt.holdon;                        # Add subsequent plots to the current axes
plt.axis([x + [-L L] y + [-L L]]);      # Set axis limits
plt.axisequal;                   # Make tick increments of each axis equal
ax1.arrow([x y], [xEnd yEnd]);     # Plot arrow
ax1.plot(xCurve, yCurve, '-k');    # Plot curve
ax1.plot(x, y, 'o', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'w');
'''
plt.show()

