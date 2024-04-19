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
    print("IMU data: {}".format(data))
#     IMU data: IMUMeasurement(frame=339878, timestamp=1424.411637, accelerometer=Vector3D(x=-0.044963, y=0.003500, z=10.641094), gyroscope=Vector3D(x=0.001911, y=0.000013, z=-0.000000), compass=4.717627)


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

    # 获取IMU传感器对象
    imu_actor = blueprint_library.filter('*imu*')[0]

    # 设置传感器数据发布频率
    # imu_sensor.set_update_rate(carla.ImuSensorProperties.UpdateRate.realtime)

    imu_sensor = world.spawn_actor(imu_actor, transform, attach_to=vehicle)  # 将传感器附在小车上
    actor_list.append(imu_sensor)

    imu_sensor.listen(lambda data: callback(data))

    time.sleep(50)

finally:
    for actor in actor_list:
        actor.destroy()
    print("All cleaned up!")
