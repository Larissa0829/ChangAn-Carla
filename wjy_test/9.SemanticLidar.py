import glob
import os
import sys
import time

'''
语义激光雷达可以看作是激光雷达和语义相机的结合，但与激光雷达有两个不同点：

激光语义雷达的返回值除了包含坐标点外，还包含入射角与表面法线之间的余弦值和CARLA对对象命名的语义标签。
激光语义雷达不包含点强度、删除点和噪声属性。
'''

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import random
import numpy as np
import cv2
from queue import Queue, Empty
import random

random.seed(50)

# args
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--host', metavar='H', default='127.0.0.1', help='IP of the host server (default: 127.0.0.1)')
parser.add_argument('--port', '-p', default=2000, type=int, help='TCP port to listen to (default: 2000)')
parser.add_argument('--tm_port', default=8000, type=int, help='Traffic Manager Port (default: 8000)')
parser.add_argument('--ego-spawn', type=list, default=None, help='[x,y] in world coordinate')
parser.add_argument('--save-path', default='存储路径', help='Synchronous mode execution')
args = parser.parse_args()

# 图片大小可自行修改
IM_WIDTH = 640
IM_HEIGHT = 480

actor_list, sensor_list = [], []
sensor_type = ['sem']


def main(args):
    # We start creating the client
    client = carla.Client(args.host, args.port)
    client.set_timeout(5.0)

    world = client.get_world()
    # world = client.load_world('Town04')

    blueprint_library = world.get_blueprint_library()
    try:
        original_settings = world.get_settings()
        settings = world.get_settings()

        # We set CARLA syncronous mode
        settings.fixed_delta_seconds = 0.05
        settings.synchronous_mode = True
        world.apply_settings(settings)
        spectator = world.get_spectator()

        # 手动规定
        # transform_vehicle = carla.Transform(carla.Location(0, 10, 0), carla.Rotation(0, 0, 0))
        # 自动选择
        transform_vehicle = random.choice(world.get_map().get_spawn_points())
        ego_vehicle = world.spawn_actor(random.choice(blueprint_library.filter("model3")), transform_vehicle)
        ego_vehicle.set_autopilot(True)
        actor_list.append(ego_vehicle)

        # -------------------------- 添加语义分割相机--------------------------#
        sensor_queue = Queue()
        sem_bp = blueprint_library.find('sensor.camera.semantic_segmentation')
        sem_bp.set_attribute("image_size_x", f"{IM_WIDTH}")
        sem_bp.set_attribute("image_size_y", f"{IM_HEIGHT}")
        sem_bp.set_attribute("fov", str(60))
        # 场视角，需要组成环视需要“360/场视角”个摄像头，这里展示两个摄像头的效果

        # Yaw（偏航）：欧拉角向量的y轴
        # Pitch（俯仰）：欧拉角向量的x轴
        # Roll（翻滚）： 欧拉角向量的z轴
        # sem_transform = carla.Transform(carla.Location(0,0,10),carla.Rotation(0,0,0))
        sem01 = world.spawn_actor(sem_bp, carla.Transform(carla.Location(0, 0, 1.8), carla.Rotation(yaw=60)),
                                  attach_to=ego_vehicle)
        sem01.listen(lambda data: sensor_callback(data, sensor_queue, "sem_01"))
        sensor_list.append(sem01)
        sem02 = world.spawn_actor(sem_bp, carla.Transform(carla.Location(0, 0, 1.8), carla.Rotation(yaw=0)),
                                  attach_to=ego_vehicle)
        sem02.listen(lambda data: sensor_callback(data, sensor_queue, "sem_02"))
        sensor_list.append(sem02)
        sem03 = world.spawn_actor(sem_bp, carla.Transform(carla.Location(0, 0, 1.8), carla.Rotation(yaw=-60)),
                                  attach_to=ego_vehicle)
        sem03.listen(lambda data: sensor_callback(data, sensor_queue, "sem_03"))
        sensor_list.append(sem03)

        # -------------------------- 设置完毕 --------------------------#

        # 设置traffic manager
        tm = client.get_trafficmanager(args.tm_port)
        tm.set_synchronous_mode(True)
        # 是否忽略红绿灯
        # tm.ignore_lights_percentage(ego_vehicle, 100)
        # 如果限速30km/h -> 30*(1-10%)=27km/h
        tm.global_percentage_speed_difference(10.0)
        ego_vehicle.set_autopilot(True, tm.get_port())

        while True:
            # Tick the server
            world.tick()

            # 将CARLA界面摄像头跟随车动
            loc = ego_vehicle.get_transform().location
            spectator.set_transform(
                carla.Transform(carla.Location(x=loc.x, y=loc.y, z=35), carla.Rotation(yaw=0, pitch=-90, roll=0)))

            w_frame = world.get_snapshot().frame
            print("\nWorld's frame: %d" % w_frame)
            try:
                sems = []
                for i in range(0, len(sensor_list)):
                    s_frame, s_name, s_data = sensor_queue.get(True, 1.0)
                    print("    Frame: %d   Sensor: %s" % (s_frame, s_name))
                    sensor_type = s_name.split('_')[0]
                    if sensor_type == 'sem':
                        sems.append(process_semantic(s_data))

                        # 仅用来可视化 可注释
                rgb = np.concatenate(sems, axis=1)[..., :3]  # 合并图像

                cv2.imshow('vizs', visualize_data(rgb))
                cv2.waitKey(100)
                if rgb is None or args.save_path is not None:
                    # 检查是否有各自传感器的文件夹
                    mkdir_folder(args.save_path)
                    filename = args.save_path + 'rgb/' + str(w_frame) + '.png'
                    cv2.imwrite(filename, np.array(rgb[..., ::-1]))

            except Empty:
                print("    Some of the sensor information is missed")

    finally:
        world.apply_settings(original_settings)
        tm.set_synchronous_mode(False)
        for sensor in sensor_list:
            sensor.destroy()
        for actor in actor_list:
            actor.destroy()
        print("All cleaned up!")


def mkdir_folder(path):
    for s_type in sensor_type:
        if not os.path.isdir(os.path.join(path, s_type)):
            os.makedirs(os.path.join(path, s_type))
    return True


def sensor_callback(sensor_data, sensor_queue, sensor_name):
    # Do stuff with the sensor_data data like save it to disk
    # Then you just need to add to the queue
    sensor_queue.put((sensor_data.frame, sensor_name, sensor_data))


# modify from world on rail code
def visualize_data(rgb, text_args=(cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)):
    canvas = np.array(rgb[..., ::-1])
    return canvas


# modify from manual control
def process_semantic(image):
    image.convert(carla.ColorConverter.CityScapesPalette)
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    return array


if __name__ == "__main__":
    try:
        main(args)
    except KeyboardInterrupt:
        print(' - Exited by user.')
