#!/usr/bin/env python

from __future__ import print_function

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import random
import sys
import carla

# try:
#     sys.path.append(
#         glob.glob(os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + '/carla/dist/carla-*%d.%d-%s.egg' % (
#             sys.version_info.major,
#             sys.version_info.minor,
#             'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
# except IndexError:
#     pass


# =================================create Client=================================
client = carla.Client(host='127.0.0.1', port=2000)
client.set_timeout(2.0)

# =================================create world=================================
world = client.get_world()
# world = client.load_world('Town04')
# get the world's blueprint
blueprint_library = world.get_blueprint_library()

# =================================change weather=================================
weather = carla.WeatherParameters(cloudiness=10.0, precipitation=10.0, fog_density=10.0)
world.set_weather(weather)

# =================================spawn actor=================================
# the car's blueprint
ego_vehicle_bp = blueprint_library.find('vehicle.bmw.grandtourer')
# give the car a color
ego_vehicle_bp.set_attribute('color', '0,0,0')
# give the car start point
transform = random.choice(world.get_map().get_spawn_points())
# spawn the car here
ego_vehicle = world.spawn_actor(ego_vehicle_bp, transform)

# handling the car's position
location = ego_vehicle.get_location()
location.x += 10.0
ego_vehicle.set_location(location)
# set to autopilot
ego_vehicle.set_autopilot(True)
# 我们可以甚至在中途将这辆车“冻住”，通过抹杀它的物理仿真
# actor.set_simulate_physics(False)

# =================================destroy actor=================================
# just one
# ego_vehicle.destroy()
# if you have many actors,save the in a list, and destroy them together
# client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])

# =================================create sensor=================================
camera_bp = blueprint_library.find('sensor.camera.rgb')
# the camera position is relate to the car's center
camera_transform = carla.Transform(carla.Location(z=2.4))
# attach the camera to the car
camera = world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle)
# define the camera's callback: each time the sensor data back, what should we do?
camera.listen(lambda image: image.save_to_disk(os.path.join('_out', '%06d.png' % image.frame)))

# =================================create lidar=================================
lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
lidar_bp.set_attribute('channels', str(32))
lidar_bp.set_attribute('points_per_second', str(90000))
lidar_bp.set_attribute('rotation_frequency', str(40))
lidar_bp.set_attribute('range', str(20))
# put the lidar on the car
lidar_location = carla.Location(0, 0, 2)
lidar_rotation = carla.Rotation(0, 0, 0)
lidar_transform = carla.Transform(lidar_location, lidar_rotation)
lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=ego_vehicle)
lidar.listen(lambda point_cloud: point_cloud.save_to_disk(os.path.join('_out', '%06d.ply' % point_cloud.frame)))

# =================================create spectator=================================
# 当我们去观察仿真界面时，我们会发现，自己的视野并不会随我们造的小车子移动，所以经常会跟丢它
while True:
    spectator = world.get_spectator()
    # get the location and roration of the spector through its transform
    transform = ego_vehicle.get_transform()
    spectator.set_transform(carla.Transform(transform.location + carla.Location(x=5,y=5,z=20), carla.Rotation(pitch=-90,yaw=10,roll=-90)))
    # carla.Rotation(pitch=Y-rotation, yaw=Z-rotation,roll=X-rotation)


# =================================create traffic sign and light=================================
#Get the traffic light affecting a vehicle
if ego_vehicle.is_at_traffic_light():
    traffic_light = ego_vehicle.get_traffic_light()
#Change a red traffic light to green
if traffic_light.get_state() == carla.TrafficLightState.Red:
    traffic_light.set_state(carla.TrafficLightState.Green)
    traffic_light.set_set_green_time(4.0)
