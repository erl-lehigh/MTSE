from math import pi
import matplotlib.pyplot as plt
from shapely.geometry import Point, LineString
from purepursuit import PurePursuit

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

lookahead =4
# input data
path = LineString([(1, 1), (8, 4)]) # given line path
speed = 3   # given vehicle speed
vehicle_cords = Point(2,3)   # initial vehicle coordinates
theta = 0  # rad

instance_of_PurePursuit = PurePursuit(path, speed,vehicle_cords, theta)

print('vehicle coords: '+ str(vehicle_cords.x) + ',' + str(vehicle_cords.y))

x,y = instance_of_PurePursuit.construct_path()

# plot the path to be tracked
ax1.plot(x,y)

ax1.set_ylim(ymin=0, ymax=10)
ax1.set_xlim(xmin=0, xmax=10)

# plot vehicle coordinate and its base point with color blue
ax1.arrow(vehicle_cords.x,vehicle_cords.y,dx=1,dy=1)
ax1.plot(vehicle_cords.x,vehicle_cords.y,'ob')

closest_pt = instance_of_PurePursuit.closest_point()

print('closest pt: ' + str(closest_pt.x) + ',' + str(closest_pt.y))

#############
#draw the vehicle orientation
#a line connecting the rear and front axle


front_pt = instance_of_PurePursuit.vehicle_front_pt()
ax1.plot(front_pt.x, front_pt.y, 'or')
vehicle_line = LineString([(vehicle_cords.x, vehicle_cords.y), (front_pt.x, front_pt.y)])
a, b = vehicle_line.xy
ax1.plot(a, b)

#############

ax1.plot(closest_pt.x, closest_pt.y, 'og')
ax1.axis('equal')

project = LineString([(vehicle_cords.x, vehicle_cords.y), (closest_pt.x, closest_pt.y)])
x1,y1 = project.xy
ax1.plot(x1,y1)

#############

future_pt = instance_of_PurePursuit.future_point()
ax1.plot(future_pt.x, future_pt.y, 'oy')

goal = LineString([(vehicle_cords.x, vehicle_cords.y), (future_pt.x, future_pt.y)])
x2,y2 = goal.xy
ax1.plot(x2,y2, color='y')

#############

r = instance_of_PurePursuit.compute_r()
curv = instance_of_PurePursuit.compute_curvature()
front_pt = instance_of_PurePursuit.vehicle_front_pt()
delta = instance_of_PurePursuit.compute_delta()
omega = instance_of_PurePursuit.compute_omega()

print('front vehicle pt: ' + str(front_pt))
print('r: ' + str(r))
print('curvature: ' + str(curv))
print('speed: ' + str(speed))
print('delta (rad): ' + str(delta))
print('omega (rad/s): ' + str(omega))

plt.show()
