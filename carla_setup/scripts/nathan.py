#!/usr/bin/env python
import glob
import os
import sys
import random
import time
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import String

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

IM_WIDTH = 640
IM_HEIGHT = 480

def process_img(image):
		i = np.array(image.raw_data)
		#print(dir(image))
		i2 = i.reshape((IM_HEIGHT, IM_WIDTH, 4))
		i3 = i2[:, :, :3] #Just the RGB values of RGBA
		cv2.imshow("", i3)
		cv2.waitKey(1)
		return i3/255.0
'''
def convert_pointcloud_carla_to_ros(frame, points):
  rosmsg = PointCloud()
  rosmsg.header.seq = frame
  rosmsg.header.stamp = rospy.Time.now()
  rosmsg.header.frame_id = 'lidar'
  for pt in points:
     new_pt = Point32()
     new_pt.x = pt[1]
     new_pt.y = pt[0]
     new_pt.z = - pt[2]
     rosmsg.points.append(new_pt)
  return rosmsg
'''
speed = 0.0
steering = 0.0

actor_list = []


def listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/carla/ego_vehicle/ackermann_cmd", AckermannDrive, drive)
    speed = AckermannDrive().speed
    steering = AckermannDrive().steering_angle
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def drive(AckermannDrive):
	speed = AckermannDrive.speed
	steering = AckermannDrive.steering_angle
	print("Current Speed: ", speed)
	print("Current Steering: ", steering)
	vehicle.apply_control(carla.VehicleControl(speed,steering))
	#actor_list.append(vehicle)

try:
	client = carla.Client("localhost", 2000)
	client.set_timeout(2.0)
	world = client.get_world()
	blueprint_library = world.get_blueprint_library()

	vehicle_bp = blueprint_library.filter("prius")[0]	
	print(vehicle_bp)

	spawn_point = random.choice(world.get_map().get_spawn_points())
	vehicle = world.spawn_actor(vehicle_bp, spawn_point)

	actor_list.append(vehicle)
	
	
	# ----------------------------------
	# Add a RGB camera to the vehicle. 
	# ----------------------------------
	cam_bp = None
	cam_bp = world.get_blueprint_library().find('sensor.camera.rgb')
	cam_bp.set_attribute("image_size_x",str(IM_WIDTH))
	cam_bp.set_attribute("image_size_y",str(IM_HEIGHT))
	cam_bp.set_attribute("fov",str(105))
	cam_location = carla.Location(2,0,1)
	cam_rotation = carla.Rotation(0,180,0)
	cam_transform = carla.Transform(cam_location,cam_rotation)
	ego_cam = world.spawn_actor(cam_bp,cam_transform,attach_to=vehicle, attachment_type=carla.AttachmentType.SpringArm)
	ego_cam.listen(lambda image: process_img(image))
	actor_list.append(ego_cam)
	'''
	# --------------
	# Add collision sensor to ego vehicle. 
	# --------------

	col_bp = world.get_blueprint_library().find('sensor.other.collision')
	col_location = carla.Location(0,0,0)
	col_rotation = carla.Rotation(0,0,0)
	col_transform = carla.Transform(col_location,col_rotation)
	ego_col = world.spawn_actor(col_bp,col_transform,attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
	def col_callback(colli):
		print("Collision detected:\n"+str(colli)+'\n')
	ego_col.listen(lambda colli: col_callback(colli))
	actor_list.append(ego_col)

	# --------------
	# Add Lane invasion sensor to ego vehicle. 
	# --------------

	lane_bp = world.get_blueprint_library().find('sensor.other.lane_invasion')
	lane_location = carla.Location(0,0,0)
	lane_rotation = carla.Rotation(0,0,0)
	lane_transform = carla.Transform(lane_location,lane_rotation)
	ego_lane = world.spawn_actor(lane_bp,lane_transform,attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
	def lane_callback(lane):
		print("Lane invasion detected:\n"+str(lane)+'\n')
	ego_lane.listen(lambda lane: lane_callback(lane))
	actor_list.append(ego_lane)

	# --------------
	# Add Obstacle sensor to ego vehicle. 
	# --------------

	obs_bp = world.get_blueprint_library().find('sensor.other.obstacle')
	obs_bp.set_attribute("only_dynamics",str(True))
	obs_location = carla.Location(0,0,0)
	obs_rotation = carla.Rotation(0,0,0)
	obs_transform = carla.Transform(obs_location,obs_rotation)
	ego_obs = world.spawn_actor(obs_bp,obs_transform,attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
	def obs_callback(obs):
		print("Obstacle detected:\n"+str(obs)+'\n')
	ego_obs.listen(lambda obs: obs_callback(obs))
	actor_list.append(ego_obs)
	'''
	'''
	# ----------------------------------
	# Add a Depth camera to the vehicle. 
	# ----------------------------------
	depth_cam = None
	depth_bp = world.get_blueprint_library().find('sensor.camera.depth')
	depth_location = carla.Location(2,0,1)
	depth_rotation = carla.Rotation(0,180,0)
	depth_transform = carla.Transform(depth_location,depth_rotation)
	depth_cam = world.spawn_actor(depth_bp,depth_transform,attach_to=vehicle, attachment_type=carla.AttachmentType.SpringArm)
	# This time, a color converter is applied to the image, to get the semantic segmentation view
	# Saves to /opt/carla-simulator/PythonAPI/examples/Depth_Sensor_Data
	depth_cam.listen(lambda image: image.save_to_disk('Depth_Sensor_Data/%.6d.jpg' % image.frame,carla.ColorConverter.LogarithmicDepth))
	actor_list.append(depth_cam)

	# ----------------------------------
	# Add a new LIDAR sensor the vehicle
	# ----------------------------------
	lidar_cam = None
	lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
	lidar_bp.set_attribute('channels',str(32))
	lidar_bp.set_attribute('points_per_second',str(90000))
	lidar_bp.set_attribute('rotation_frequency',str(40))
	lidar_bp.set_attribute('range',str(20))
	lidar_location = carla.Location(0,0,2)
	lidar_rotation = carla.Rotation(0,0,0)
	lidar_transform = carla.Transform(lidar_location,lidar_rotation)
	lidar_sen = world.spawn_actor(lidar_bp,lidar_transform,attach_to=vehicle,attachment_type=carla.AttachmentType.SpringArm)
	# Saves to /opt/carla-simulator/PythonAPI/examples/Lidar_Data
	lidar_sen.listen(lambda point_cloud: point_cloud.save_to_disk("Lidar_Data/%.6d.ply" % point_cloud.frame))

	'''
	listener()
	time.sleep(120)

finally:
	for actor in actor_list:
		actor.destroy()
	print("Cleaned Up!")

