# import abc
# import glob
# import os
# import sys
# from types import LambdaType
# from collections import deque
# from collections import namedtuple
#
# from matplotlib import pyplot as plt
#
# try:
#     # 输入存放carla环境的路径
#     sys.path.append(glob.glob('.../WindowsNoEditor/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
#         sys.version_info.major,
#         sys.version_info.minor,
#         'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
# except IndexError:
#     pass
#
# import carla
# import random
# import time
# import numpy as np
# import cv2
# import math
#
# IM_WIDTH = 640
# IM_HEIGHT = 480
# SHOW_PREVIEW = True
#
# SHOW_CAM = SHOW_PREVIEW
# im_width = IM_WIDTH
# im_height = IM_HEIGHT
#
# '''
#
# '''
#
# def process_img(image):
#     i = np.array(image.raw_data)
#     i2 = i.reshape((im_height, im_width, 4))
#     i3 = i2[:, :, : 3]
#     if SHOW_CAM:
#         cv2.imshow("", i3)
#         cv2.waitKey(1)
#
# def process_radar(mesure):
#     global radar_data
#     radar_data = mesure
#     print("Radar raw_data: {}".format(mesure))
#     # Radar raw_data: RadarMeasurement(frame=559050, timestamp=4630.390192, point_count=61)
#     # 暂时对雷达信号的数据特征不太了解，所以在监听时先将其保存到一个全局变量radar_data中，单独分析，现在radar_data就是采集到的雷达信号的raw_data，需要对其进行进一步分析。
#     # 使用文档中提到的转换方式:
#     points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
#     points = np.reshape(points, (len(radar_data), 4))
#     # points变量是一个n*4的矩阵，每一行代表一个被检测到的物体点，n会随每秒发射的点数变化，点数相同时，n也会有一些随机变动。
#     # 比如我设置的雷达参数是，每秒钟发射1500个点，反馈的点数是28个，points变量的值大致如下：
#     print("Radar points: {}".format(points))
#     # Radar points:
#     # [[-1.02809572e-05 -2.92968866e-03 -1.24937659e-02  4.81451569e+01]
#     #  [-9.38001176e-05 -1.43156111e-01 -1.14234313e-01  5.27695131e+00]
#     # ……
#     #  [-2.22319504e-05  8.15699697e-02 -2.70196050e-02  2.22642746e+01]]
#     # 第一列是物体相对于传感器的速度（由于当前小车静止，所以相对速度是0），
#     # 第二列是物体位置相对于传感器的方位角（弧度），
#     # 第三列是物体位置相对于传感器的高度角（弧度），
#     # 第四列是物体距离传感器的直线距离。
#     # 有了这些数据，就可以将这些点阵在空间中的x, y, z坐标计算出来:
#     l = np.cos(points[:, 2]) * points[:, 3]
#     z = np.sin(points[:, 2]) * points[:, 3]
#     y = np.cos(points[:, 1]) * l
#     x = np.sin(points[:, 1]) * l
#
#     # plt.figure("3D Scatter", facecolor="lightgray", figsize=(20, 20), dpi=80)
#     # ax3d = plt.gca(projection="3d")
#     #
#     # ax3d.scatter(x, y, z, s=10, cmap="jet", marker="o")
#     # ax3d.view_init(elev=0, azim=-70)
#     # # ax3d.set_yticks(np.arange(0, 100, 10))
#     # plt.show(block=False)
#     # plt.pause(10)
#     # plt.close()  # 关闭图形窗口
#     radar_data.save_to_disk(f'radar/%08d' % radar_data.frame)
#
# actor_list = []
#
# try:
#     # 与服务器建立连接
#     client = carla.Client('localhost', 2000)
#     client.set_timeout(10.0)
#     world = client.get_world()
#     blueprint_library = world.get_blueprint_library()
#     map = world.get_map()
#     model_3 = blueprint_library.filter('model3')[0]
#
#     actor_list = []
#     spawn_points = map.get_spawn_points()
#
#     transform = random.choice(spawn_points)  # 随机选择
#     vehicle = world.spawn_actor(model_3, transform)
#     vehicle.set_autopilot(True)
#     actor_list.append(vehicle)
#
#     # 添加一个depth相机
#     cam = blueprint_library.find('sensor.camera.depth')
#     cam.set_attribute('image_size_x', f'{im_width}')
#     cam.set_attribute('image_size_y', f'{im_height}')
#     cam.set_attribute('fov', f'110')
#
#     cam_location = carla.Location(-5, 0, 4)
#     cam_rotation = carla.Rotation(pitch=-10, yaw=0, roll=0.0)  # 若不设置角度当前传感器与车头前向保持一致
#     cam_transform = carla.Transform(cam_location, cam_rotation)
#
#     cam_sensor = world.spawn_actor(cam, transform, attach_to=vehicle)  # 将传感器附在小车上
#     actor_list.append(cam_sensor)
#     cam_sensor.listen(lambda data: process_img(data))  # 传感器开始监听,并显示图片
#
#     # 获取radar传感器对象
#     radar_actor = blueprint_library.filter('*radar*')[0]
#
#     radar_actor.set_attribute('points_per_second', '5000')
#
#     radar_sensor = world.spawn_actor(radar_actor, transform, attach_to=vehicle)  # 将传感器附在小车上
#     actor_list.append(radar_sensor)
#
#     # 设置雷达发射点数为5000
#
#     # 设置传感器数据发布频率
#     # radar_sensor.set_update_rate(carla.radarSensorProperties.UpdateRate.realtime)
#
#     # 雷达监听
#     radar_sensor.listen(lambda data: process_radar(data))
#
#
#
#     time.sleep(50)
#
# finally:
#     for actor in actor_list:
#         actor.destroy()
#     print("All cleaned up!")


import sys
import time

import carla
import open3d as o3d
import numpy as np
import random
from matplotlib import cm
from datetime import datetime

VIDIDIS = np.array(cm.get_cmap("plasma").colors)
VID_RANGE = np.linspace(0.0, 1.0, VIDIDIS.shape[0])
LABEL_COLORS = np.array([
    (255, 255, 255),  # None
    (70, 70, 70),  # Building
    (100, 40, 40),  # Fences
    (55, 90, 80),  # Other
    (220, 20, 60),  # Pedestrian
    (153, 153, 153),  # Pole
    (157, 234, 50),  # RoadLines
    (128, 64, 128),  # Road
    (244, 35, 232),  # Sidewalk
    (107, 142, 35),  # Vegetation
    (0, 0, 142),  # Vehicle
    (102, 102, 156),  # Wall
    (220, 220, 0),  # TrafficSign
    (70, 130, 180),  # Sky
    (81, 0, 81),  # Ground
    (150, 100, 100),  # Bridge
    (230, 150, 140),  # RailTrack
    (180, 165, 180),  # GuardRail
    (250, 170, 30),  # TrafficLight
    (110, 190, 160),  # Static
    (170, 120, 50),  # Dynamic
    (45, 60, 150),  # Water
    (145, 170, 100),  # Terrain
]) / 255.0  # normalize each channel [0-1] since is what Open3D uses



def generate_lidar_bp(blueprint_library, delta):
    """
    To get lidar bp
    :param blueprint_library: the world blueprint_library
    :param delta: update rate(s)
    :return: lidar bp
    """
    lidar_bp = blueprint_library.find("sensor.lidar.ray_cast_semantic")
    # lidar_bp.set_attribute("dropoff_general_rate", "0.0")
    # lidar_bp.set_attribute("dropoff_intensity_limit", "1.0")
    # lidar_bp.set_attribute("dropoff_zero_intensity", "0.0")

    lidar_bp.set_attribute("upper_fov", str(15.0))
    lidar_bp.set_attribute("lower_fov", str(-25.0))
    lidar_bp.set_attribute("channels", str(64.0))
    lidar_bp.set_attribute("range", str(100.0))
    lidar_bp.set_attribute("rotation_frequency", str(1.0 / delta))
    lidar_bp.set_attribute("points_per_second", str(500000))

    return lidar_bp


def lidar_callback(point_cloud, point_list):
    # We need to convert point cloud(carla-format) into numpy.ndarray
    data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype("f4")))
    data = np.reshape(data, (int(data.shape[0] / 6), 6))

    intensity = data[:, -1]
    intensity_col = 1.0 - np.log(intensity) / np.log(np.exp(-0.004 * 100))
    int_color = np.c_[
        np.interp(intensity_col, VID_RANGE, VIDIDIS[:, 0]),
        np.interp(intensity_col, VID_RANGE, VIDIDIS[:, 1]),
        np.interp(intensity_col, VID_RANGE, VIDIDIS[:, 2])]

    points = data[:, :-1]  # we only use x, y, z coordinates
    points[:, 1] = -points[:, 1]  # This is different from official script
    point_list.points = o3d.utility.Vector3dVector(points[:, :3])
    point_list.colors = o3d.utility.Vector3dVector(int_color)
    print("_________")
    # 保存PointCloud对象到PLY文件
    o3d.io.write_point_cloud(f"lidarSem/{point_cloud.frame}.ply", point_list)
    print("___22222222______")


def lidarSem_callback(point_cloud, semantic_data, point_list):
    # data = np.copy(np.frombuffer(point_cloud.raw_data, dtype=np.dtype("f4")))
    # data = np.reshape(data, (int(data.shape[0] / 6), 6))
    # # 获取强度值
    # intensity = data[:, 2]  # 假设强度是第三列
    # # 归一化强度值到 0-1 范围
    # intensity_normalized = (intensity - np.min(intensity)) / (np.max(intensity) - np.min(intensity))
    # # 使用归一化强度值来插值颜色
    # colors = np.c_[
    #     np.interp(intensity_normalized, VID_RANGE, VIDIDIS[:, 0]),
    #     np.interp(intensity_normalized, VID_RANGE, VIDIDIS[:, 1]),
    #     np.interp(intensity_normalized, VID_RANGE, VIDIDIS[:, 2])]
    # point_list.points = o3d.utility.Vector3dVector(data[:, :3])
    # point_list.colors = o3d.utility.Vector3dVector(colors * 255)

    data = np.frombuffer(point_cloud.raw_data, dtype=np.dtype([
        ('x', np.float32), ('y', np.float32), ('z', np.float32),
        ('CosAngle', np.float32), ('ObjIdx', np.uint32), ('ObjTag', np.uint32)]))
    points = np.array([data['x'], -data['y'], data['z']]).T
    labels = np.array(data['ObjTag'][(data['ObjTag'] <= 23) & (data['ObjTag'] >= 0)] - 1)
    int_color = LABEL_COLORS[labels]
    print(int_color)
    point_list.points = o3d.utility.Vector3dVector(points)
    point_list.colors = o3d.utility.Vector3dVector(int_color*255)

    print("_________")
    # 保存PointCloud对象到PLY文件
    o3d.io.write_point_cloud(f"lidarSem/{point_cloud.frame}.ply", point_list)
    print("__-33333333333333________")
    input("hhh")


if __name__ == "__main__":
    print(f"Let's show point cloud with open3d in carla!")
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(2.0)
    world = client.get_world()

    try:
        # 1. To do some synchronous settings in world
        original_settings = world.get_settings()
        settings = world.get_settings()
        traffic_manager = client.get_trafficmanager(8000)
        traffic_manager.set_synchronous_mode(True)

        delta = 0.05

        settings.fixed_delta_seconds = delta
        settings.synchronous_mode = True
        # settings.no_rendering_mode = True
        world.apply_settings(settings)

        # 2. To get blueprint for your ego vehicle and spawn it!
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter("model3")[0]
        vehicle_transform = random.choice(world.get_map().get_spawn_points())
        vehicle = world.spawn_actor(vehicle_bp, vehicle_transform)
        vehicle.set_autopilot(True)

        # 3. To get lidar blueprint and spawn it on your car!
        lidar_bp = generate_lidar_bp(blueprint_library, delta)
        lidar_transform = carla.Transform(carla.Location(x=-0.5, z=1.8))
        lidar = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)

        # 4. We set a point_list to store our point cloud
        point_list = o3d.geometry.PointCloud()

        # 5. Listen to the lidar to collect point cloud
        # lidar.listen(lambda data: lidar_callback(data, point_list))
        lidar.listen(lambda data: lidarSem_callback(data, data, point_list))
        # lidar.listen(lambda data: data.save_to_disk(f'lidarSem/%08d' % data.frame))

        # 6. We set some basic settings for display with open3d
        vis = o3d.visualization.Visualizer()
        vis.create_window(
            window_name="Display Point Cloud",
            width=960,
            height=540,
            left=480,
            top=270)

        vis.get_render_option().background_color = [0.05, 0.05, 0.05]
        vis.get_render_option().point_size = 1
        vis.get_render_option().show_coordinate_frame = True

        frame = 0
        dt0 = datetime.now()

        while True:
            if frame == 2:
                vis.add_geometry(point_list)

            vis.update_geometry(point_list)
            vis.poll_events()
            vis.update_renderer()
            time.sleep(0.005)

            world.tick()

            # We here add a spectator to watch how our ego vehicle will move
            spectator = world.get_spectator()
            transform = vehicle.get_transform()  # we get the transform of vehicle
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=50),
                                                    carla.Rotation(pitch=-90)))

            process_time = datetime.now() - dt0
            sys.stdout.write("\r" + "FPS: " + str(1.0 / process_time.total_seconds()) + "Current Frame: " + str(frame))
            sys.stdout.flush()
            dt0 = datetime.now()

            frame += 1

    finally:
        world.apply_settings(original_settings)
        traffic_manager.set_synchronous_mode(False)
        vehicle.destroy()
        lidar.destroy()
        vis.destroy_window()
