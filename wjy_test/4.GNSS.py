import abc
import glob
import os
import sys
from types import LambdaType
from collections import deque
from collections import namedtuple

try:
    # 输入存放carla环境的路径
    sys.path.append(glob.glob('.../WindowsNoEditor/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import random
import time
import numpy as np
import cv2
import math

IM_WIDTH = 640
IM_HEIGHT = 480
SHOW_PREVIEW = True

SHOW_CAM = SHOW_PREVIEW
im_width = IM_WIDTH
im_height = IM_HEIGHT

'''
报告物体的当前全球导航卫星系统位置，通过将度量位置添加到地图初始地理参考位置计算。
 '''


def callback(data):
    print("GNSS data: {}".format(data))
    # GnssMeasurement(frame=344170, timestamp=1489.016494, lat=0.001193, lon=-0.000765, alt=0.412847)
    global car_longitude, car_latitude, car_altitude
    print(
        f">>>>>>gnss传感器geo坐标值: longitude 经度:{data.longitude},latitude 维度:{data.latitude},altitude 海拔:{data.altitude}")
    car_longitude = data.longitude
    car_latitude = data.latitude
    car_altitude = data.altitude
    car_transform = data.transform
    print(
        f">>>>>>gnss传感器中的carla坐标值 x:{car_transform.location.x},y:{car_transform.location.y},z:{car_transform.location.z}")



actor_list = []

try:
    # 与服务器建立连接
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    map = world.get_map()
    model_3 = blueprint_library.filter('model3')[0]

    actor_list = []
    spawn_points = map.get_spawn_points()

    transform = random.choice(spawn_points) # 随机选择
    vehicle = world.spawn_actor(model_3, transform)
    vehicle.set_autopilot(True)
    actor_list.append(vehicle)

    # 导航卫星传感器(Gnss sensor)
    gnss_actor  = blueprint_library.filter('*gnss*')[0]

    gnss_location = carla.Location(0, 0, 0)
    gnss_rotation = carla.Rotation(pitch=-10, yaw=0, roll=0.0)  # 若不设置角度当前传感器与车头前向保持一致
    gnss_transform = carla.Transform(gnss_location, gnss_rotation)

    gnss_sensor = world.spawn_actor(gnss_actor, transform, attach_to=vehicle)  # 将传感器附在小车上
    actor_list.append(gnss_sensor)

    car_longitude = 0.0
    car_latitude = 0.0
    car_altitude = 0.0
    car_transform = None

    gnss_sensor.listen(lambda data: callback(data))



    time.sleep(50)

finally:
    for actor in actor_list:
        actor.destroy()
    print("All cleaned up!")
