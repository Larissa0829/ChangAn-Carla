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
后定义rgb相机，将其附着在小车上。还需要单独写一个函数（process_img）来处理接收的图像并将其显示在cv图像框中。
'''

recordImage_filepath="images"
recordDepth_filepath="depth"
def process_img(image):
    i = np.array(image.raw_data)
    i2 = i.reshape((im_height, im_width, 4))
    i3 = i2[:, :, : 3]
    if SHOW_CAM:
        cv2.imshow("", i3)
        cv2.waitKey(1)
        image.save_to_disk(f'{recordImage_filepath}/%08d' % image.frame)

def process_depth_img(image):
    #显示图像前加一步转换可以显示灰度图像
    #image.convert(carla.ColorConverter.Depth)#灰度图像
    #image.convert(carla.ColorConverter.LogarithmicDepth)#对数灰度图像
    i = np.array(image.raw_data)
    i2 = i.reshape((im_height, im_width , 4))
    i3 = i2[: , : , : 3]
    if SHOW_CAM:
        cv2.imshow("depth",i3)
        cv2.waitKey(1)
        image.save_to_disk(f'{recordDepth_filepath}/%08d' % image.frame)


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

    transform = random.choice(spawn_points)  # 随机选择
    vehicle = world.spawn_actor(model_3, transform)
    vehicle.set_autopilot(True)
    actor_list.append(vehicle)

    # 添加一个rgb相机
    cam = blueprint_library.find('sensor.camera.rgb')
    cam.set_attribute('image_size_x', f'{im_width}')
    cam.set_attribute('image_size_y', f'{im_height}')
    cam.set_attribute('fov', f'110')

    cam_location = carla.Location(-5, 0, 4)
    cam_rotation = carla.Rotation(pitch=-10, yaw=0, roll=0.0)  # 若不设置角度当前传感器与车头前向保持一致
    cam_transform = carla.Transform(cam_location, cam_rotation)

    cam_sensor = world.spawn_actor(cam, cam_transform, attach_to=vehicle)  # 将传感器附在小车上
    actor_list.append(cam_sensor)

    cam_sensor.listen(lambda data: process_img(data))  # 传感器开始监听,并显示图片
    actor_list.append(cam_sensor)

    focal_distance = cam.get_attribute("focal_distance").recommended_values[0]
    image_size_x = cam.get_attribute("image_size_x").recommended_values[0]

    print("focal_distance:",focal_distance)
    print("image_size_x:",image_size_x)

    # 添加一个depth相机
    camdepth = blueprint_library.find('sensor.camera.depth')
    camdepth.set_attribute('image_size_x', f'{im_width}')
    camdepth.set_attribute('image_size_y', f'{im_height}')
    camdepth.set_attribute('fov', f'110')

    cam_location = carla.Location(-5, 0, 4)
    cam_rotation = carla.Rotation(pitch=-10, yaw=0, roll=0.0)  # 若不设置角度当前传感器与车头前向保持一致
    cam_transform = carla.Transform(cam_location, cam_rotation)

    camdepth_sensor = world.spawn_actor(camdepth, cam_transform, attach_to=vehicle)  # 将传感器附在小车上
    actor_list.append(camdepth_sensor)

    camdepth_sensor.listen(lambda data: process_depth_img(data))  # 传感器开始监听,并显示图片
    actor_list.append(camdepth_sensor)

    time.sleep(50)

finally:
    for actor in actor_list:
        actor.destroy()
    print("All cleaned up!")
