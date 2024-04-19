import glob
import os
import sys
import time
import random
import time
import numpy as np
import cv2

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from carla import Transform, Location, Rotation

IM_WIDTH = 640
IM_HEIIGHT = 480


def process_semantic(image):
    # image.convert(carla.ColorConverter.CityScapesPalette)
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    cv2.imshow("", array)
    cv2.waitKey(20)
    return array / 255.0


actor_list = []
try:
    # 连接master
    client = carla.Client('localhost', 2000)
    client.set_timeout(5.0)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    bp = blueprint_library.filter("model3")[0]
    spawn_point = Transform(Location(x=54.469772, y=-64.348633, z=0.600000), Rotation(pitch=0.000000, yaw=179.976562, roll=0.000000))
    vehicle = world.spawn_actor(bp, spawn_point)

    vehicle.set_autopilot(enabled=True)
    actor_list.append(vehicle)
    # 添加一个语义分割相机
    sem_cam = None
    sem_bp = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
    # sem_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    sem_bp.set_attribute("image_size_x", f"{IM_WIDTH}")
    sem_bp.set_attribute("image_size_y", f"{IM_HEIIGHT}")
    sem_bp.set_attribute("fov", str(105))
    sem_location = carla.Location(-5, 0, 2.5)
    sem_rotation = carla.Rotation(0, 0, 0)
    sem_transform = carla.Transform(sem_location, sem_rotation)
    sem_cam = world.spawn_actor(sem_bp, sem_transform, attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
    actor_list.append(sem_cam)
    # 监听相机，并显示图像
    sem_cam.listen(lambda image: process_semantic(image))
    time.sleep(50)

finally:
    for actor in actor_list:
        actor.destroy()
    print("All cleaned up!")