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
该相机对视野中的不同物体打上不同的标签，传回的数据自带标签类别（存储在图像的red通道中），
可以用carla.Image类自带的函数将其转化为用不同颜色表示不同物体的图像。


 '''


def process_img(image):
    # 除了更改实体类名称外，语义分割相机的数据图像在显示前需要对图像进行转换
    image.convert(carla.ColorConverter.CityScapesPalette)
    i = np.array(image.raw_data)
    i2 = i.reshape((im_height, im_width , 4))
    i3 = i2[: , : , : 3]
    if SHOW_CAM:
        cv2.imshow("",i3)
        cv2.waitKey(1)
        image.save_to_disk(f'semantic/%08d' % image.frame)


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

    # 添加一个rgb相机
    cam = blueprint_library.find('sensor.camera.semantic_segmentation')
    cam.set_attribute('image_size_x', f'{im_width}')
    cam.set_attribute('image_size_y', f'{im_height}')
    cam.set_attribute('fov', f'110')

    cam_location = carla.Location(-5, 0, 4)
    cam_rotation = carla.Rotation(pitch=-10, yaw=0, roll=0.0)  # 若不设置角度当前传感器与车头前向保持一致
    cam_transform = carla.Transform(cam_location, cam_rotation)

    cam_sensor = world.spawn_actor(cam, transform, attach_to=vehicle)  # 将传感器附在小车上
    actor_list.append(cam_sensor)
    cam_sensor.listen(lambda data: process_img(data))  # 传感器开始监听,并显示图片
    time.sleep(10)

finally:
    for actor in actor_list:
        actor.destroy()
    print("All cleaned up!")
