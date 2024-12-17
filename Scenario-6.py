import glob
import os
import sys
import math
import time
import numpy as np
import cv2
import random
from collections import deque

#Global Setting
ROUNDING_FACTOR = 2
SECONDS_PER_EPISODE = 120
# LIMIT_RADAR = 600
np.random.seed(32)
random.seed(32)
# MAX_LEN = 600
velodyne_data = [10]
x_cord=0
y_cord=0
z_cord=0
h_angle=0
COLLISION_DIST = 0

try:
	sys.path.append(glob.glob('../../../carla/dist/carla-*%d.%d-%s.egg' % (
		sys.version_info.major,
		sys.version_info.minor,
		'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
	pass
import carla

class CarlaVehicle(object):
	"""
	class responsable of:
		-spawning the ego vehicle
		-destroy the created objects
		-providing environment for RL training
	"""

	#Class Variables
	def __init__(self):
		self.client = carla.Client('localhost',2000)
		self.client.set_timeout(10.0)
		self.radar_data = deque(maxlen=400)
		self.x=5

	def reset(self,Norender):
		'''reset function to reset the environment before 
		the begining of each episode
		:params Norender: to be set true during training
		'''
		global velodyne_data
		global x_cord,y_cord,z_cord,h_angle
		velodyne_data[0]=10
		self.collision_hist = []
		self.world = self.client.get_world()
		self.map = self.world.get_map()
		'''target_location: Orientation details of the target loacation
		(To be obtained from route planner)'''
		# self.target_waypoint = carla.Transform(carla.Location(x = 1.89, y = 117.06, z=0), carla.Rotation(yaw=269.63))

		#Code for setting no rendering mode
		if Norender:
			settings = self.world.get_settings()
			settings.no_rendering_mode = False
			self.world.apply_settings(settings)

		self.actor_list = []
		self.blueprint_library = self.world.get_blueprint_library()
		self.bp = self.blueprint_library.filter("model3")[0]
		self.bp1 = self.blueprint_library.filter("prius")[0]

		# self.static_prop = self.blueprint_library.filter("static.prop.streetbarrier")[0]

		#create ego vehicle the reason for adding offset to z is to avoid collision
		#right turning agent
		#training
		# off = random.uniform(0, 3)
		#testing
		off = 3
		init_loc = carla.Location(x = 5.8, y = 142.6, z=2)
		self.init_pos = carla.Transform(init_loc, carla.Rotation(yaw=-90))
		init_loc1 = carla.Location(x = 38, y = 134, z=2)
		self.init_pos1 = carla.Transform(init_loc1, carla.Rotation(yaw=0))
		init_loc2 = carla.Location(x = 38, y = 130, z=2)
		self.init_pos2 = carla.Transform(init_loc2, carla.Rotation(yaw=0))
		# init_loc2 = carla.Location(x = -142, y = -11, z=2)
		# self.init_pos2 = carla.Transform(init_loc2, carla.Rotation(yaw=-90))
		x_cord=self.init_pos.location.x
		y_cord=self.init_pos.location.y
		z_cord=self.init_pos.location.z
		h_angle=self.init_pos.rotation.yaw
		# self.target_waypoint = carla.Transform(carla.Location(x = 1.89, y = 117.06, z=0), carla.Rotation(yaw=269.63))
	
		# self.target_waypoint = carla.Transform(carla.Location(x = -135.0, y = -3 + off, z=0), carla.Rotation(yaw=0))

		#left lane change
		#init_loc = carla.Location(x = 50.52, y = 135.91, z=1)
		#self.target_waypoint = carla.Transform(carla.Location(x = 1.89, y = 117.06, z=0), carla.Rotation(yaw=269.63))	
		
		self.vehicle = self.world.spawn_actor(self.bp, self.init_pos)
		self.actor_list.append(self.vehicle)
		self.vehicle1 = self.world.spawn_actor(self.bp1, self.init_pos1)
		self.actor_list.append(self.vehicle1)

		control =  carla.VehicleControl()
		control.steer = 0
		control.throttle = 0.12
		self.vehicle1.apply_control(control)


		self.vehicle2 = self.world.spawn_actor(self.bp1, self.init_pos2)
		self.actor_list.append(self.vehicle2)

		control =  carla.VehicleControl()
		control.steer = 0
		control.throttle = 0.22
		self.vehicle2.apply_control(control)
		# self.vehicle2 = self.world.spawn_actor(self.bp, self.init_pos2)
		# self.actor_list.append(self.vehicle2)
		