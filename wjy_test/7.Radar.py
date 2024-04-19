import abc
import glob
import os
import sys
from types import LambdaType
from collections import deque
from collections import namedtuple

from matplotlib import pyplot as plt

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
传感器创建一个圆锥视图，该视图被转换为视线中元素及其相对于传感器的速度的 2D 点图。这可用于塑造元素并评估其运动和方向。由于使用了极坐标，这些点将集中在视图中心周围。

测量的点包含在 carla 中。RadarMeasurement 作为 carla 的数组。RadarDetection，指定它们的极坐标、距离和速度。 雷达传感器提供的原始数据可以很容易地转换为numpy可管理的格式：

# To get a numpy [[vel, azradarth, altitude, depth],...[,,,]]:
points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
points = np.reshape(points, (len(radar_data), 4))

提供的脚本使用此传感器来显示正在检测的点，并在静态时将其涂成白色，在向物体移动时将其涂成红色，在远离对象时将其涂成蓝色
'''


def process_radar(mesure):
    global radar_data
    radar_data = mesure
    print("Radar raw_data: {}".format(mesure))
    # Radar raw_data: RadarMeasurement(frame=559050, timestamp=4630.390192, point_count=61)
    # 暂时对雷达信号的数据特征不太了解，所以在监听时先将其保存到一个全局变量radar_data中，单独分析，现在radar_data就是采集到的雷达信号的raw_data，需要对其进行进一步分析。
    # 使用文档中提到的转换方式:
    points = np.frombuffer(radar_data.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (len(radar_data), 4))
    # points变量是一个n*4的矩阵，每一行代表一个被检测到的物体点，n会随每秒发射的点数变化，点数相同时，n也会有一些随机变动。
    # 比如我设置的雷达参数是，每秒钟发射1500个点，反馈的点数是28个，points变量的值大致如下：
    print("Radar points: {}".format(points))
    # Radar points:
    # [[-1.02809572e-05 -2.92968866e-03 -1.24937659e-02  4.81451569e+01]
    #  [-9.38001176e-05 -1.43156111e-01 -1.14234313e-01  5.27695131e+00]
    # ……
    #  [-2.22319504e-05  8.15699697e-02 -2.70196050e-02  2.22642746e+01]]
    # 第一列是物体相对于传感器的速度（由于当前小车静止，所以相对速度是0），
    # 第二列是物体位置相对于传感器的方位角（弧度），
    # 第三列是物体位置相对于传感器的高度角（弧度），
    # 第四列是物体距离传感器的直线距离。
    # 有了这些数据，就可以将这些点阵在空间中的x, y, z坐标计算出来:
    l = np.cos(points[:, 2]) * points[:, 3]
    z = np.sin(points[:, 2]) * points[:, 3]
    y = np.cos(points[:, 1]) * l
    x = np.sin(points[:, 1]) * l

    plt.figure("3D Scatter", facecolor="lightgray", figsize=(20, 20), dpi=80)
    ax3d = plt.gca(projection="3d")

    ax3d.scatter(x, y, z, s=10, cmap="jet", marker="o")
    ax3d.view_init(elev=0, azim=-70)
    # ax3d.set_yticks(np.arange(0, 100, 10))
    plt.show(block=False)
    plt.pause(10)
    plt.close()  # 关闭图形窗口

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

    # 获取radar传感器对象
    radar_actor = blueprint_library.filter('*radar*')[0]

    radar_actor.set_attribute('points_per_second', '5000')

    radar_sensor = world.spawn_actor(radar_actor, transform, attach_to=vehicle)  # 将传感器附在小车上
    actor_list.append(radar_sensor)

    # 设置雷达发射点数为5000

    # 设置传感器数据发布频率
    # radar_sensor.set_update_rate(carla.radarSensorProperties.UpdateRate.realtime)

    # 雷达监听
    radar_sensor.listen(lambda data: process_radar(data))

    time.sleep(50)

finally:
    for actor in actor_list:
        actor.destroy()
    print("All cleaned up!")
