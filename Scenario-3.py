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
SECONDS_PER_EPISODE = 30
# LIMIT_RADAR = 500
np.random.seed(32)
random.seed(32)
# MAX_LEN = 500
velodyne_data = [10]
x_cord=0
y_cord=0
z_cord=0
h_angle=0
COLLISION_DIST = 5

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
		self.client.set_timeout(5.0)
		self.radar_data = deque(maxlen=600)

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

		self.static_prop = self.blueprint_library.filter("static.prop.streetbarrier")[0]


		#==================For learning Lane Change Maneuver===============
		# prop_1_loc = carla.Transform(carla.Location(x=69.970375, y=133.594208, z=0.014589), carla.Rotation(pitch=0.145088, yaw=-0.817204+90, roll=0.000000))
		# prop_2_loc = carla.Transform(carla.Location(x=67.970741, y=133.622726, z=0.009964), carla.Rotation(pitch=0.119906, yaw=-0.817204+90, roll=0.000000))
		# prop_3_loc = carla.Transform(carla.Location(x=83, y=130.111465, z=0.009964), carla.Rotation(pitch=0.119906, yaw=-0.817204+90, roll=0.000000))
		# prop_1 = self.world.spawn_actor(self.static_prop, prop_1_loc)
		# prop_2 = self.world.spawn_actor(self.static_prop, prop_2_loc)
		# prop_3 = self.world.spawn_actor(self.static_prop, prop_3_loc)
		# self.actor_list.append(prop_1)
		# self.actor_list.append(prop_2)
		# self.actor_list.append(prop_3)

		# These will be in left side of the road
		# prop_lt_1_loc = carla.Transform(carla.Location(x=96, y=126, z=0.014589), carla.Rotation(pitch=0.145088, yaw=-0.817204, roll=0.000000))
		# prop_lt_2_loc = carla.Transform(carla.Location(x=100, y=126, z=0.009964), carla.Rotation(pitch=0.119906, yaw=-0.817204, roll=0.000000))
		# prop_lt_3_loc = carla.Transform(carla.Location(x=104, y=126, z=0.009964), carla.Rotation(pitch=0.119906, yaw=-0.817204, roll=0.000000))
		# prop_lt_4_loc = carla.Transform(carla.Location(x=108, y=126, z=0.009964), carla.Rotation(pitch=0.119906, yaw=-0.817204, roll=0.000000))
		# prop_lt_5_loc = carla.Transform(carla.Location(x=112, y=126, z=0.009964), carla.Rotation(pitch=0.119906, yaw=-0.817204, roll=0.000000))
		# prop_lt_1 = self.world.spawn_actor(self.static_prop, prop_lt_1_loc)
		# prop_lt_2 = self.world.spawn_actor(self.static_prop, prop_lt_2_loc)
		# prop_lt_3 = self.world.spawn_actor(self.static_prop, prop_lt_3_loc)
		# prop_lt_4 = self.world.spawn_actor(self.static_prop, prop_lt_4_loc)
		# prop_lt_5 = self.world.spawn_actor(self.static_prop, prop_lt_5_loc)

		# These will be the end points on the road
		# prop_end_rt_loc = carla.Transform(carla.Location(x=106, y=133, z=0.014589), carla.Rotation(pitch=0.094270, yaw=0, roll=-1.246277))
		# prop_start_lt_loc = carla.Transform(carla.Location(x=92, y=129.5, z=1), carla.Rotation(pitch=0.094270, yaw=0, roll=-1.246277))
		# prop_start_lt = self.world.spawn_actor(self.bp, prop_start_lt_loc)
		# vehicle2 = self.world.spawn_actor(self.bp, prop_end_rt_loc)
		

		# It will bw on right side of the road
		# prop_rt_1_loc = carla.Transform(carla.Location(x=98, y=136.5, z=1), carla.Rotation(pitch=0.094270, yaw=0, roll=-1.246277))
		# prop_rt_1 = self.world.spawn_actor(self.bp, prop_rt_1_loc)

		init_loc1 = carla.Location(x = 101, y = 133, z=1)
		self.init_pos1 = carla.Transform(init_loc1, carla.Rotation(pitch=0.094270, yaw=0, roll=-1.246277))
		self.vehicle1 = self.world.spawn_actor(self.bp1, self.init_pos1)
		self.actor_list.append(self.vehicle1)

		control =  carla.VehicleControl()
		control.steer = 0
		control.throttle = 0.17
		self.vehicle1.apply_control(control)

		init_loc2 = carla.Location(x = 101, y = 129, z=1)
		self.init_pos2 = carla.Transform(init_loc2, carla.Rotation(pitch=0.094270, yaw=0, roll=-1.246277))
		self.vehicle2 = self.world.spawn_actor(self.bp1, self.init_pos2)
		self.actor_list.append(self.vehicle2)

		control =  carla.VehicleControl()
		control.steer = 0
		control.throttle = 0.27
		self.vehicle2.apply_control(control)


