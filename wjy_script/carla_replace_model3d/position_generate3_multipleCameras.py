import glob
import os
import sys

'''
生产所有完整的数据
camera_dic = {
    "rgb": 1,
    "semantic": 1,
    "depth": 0,
    "lidarSem": 1,
}
'''

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from carla import Transform, Location, Rotation
import random
import queue
import numpy as np
import cv2
import json
import copy
import open3d as o3d
from matplotlib import cm
from datetime import datetime

VIDIDIS = np.array(cm.get_cmap("plasma").colors)
VID_RANGE = np.linspace(0.0, 1.0, VIDIDIS.shape[0])

# 生成的对象列表
actor_list = []
# 位姿输出文件路径
experience_time = "3"  # 实验次数，也可以代表场景次数
position_filepath = experience_time + '/position/position.json'
recordImage_filepath = experience_time + '/position/images'
recordSemantic_filepath = experience_time + '/position/semantic'
recordDepth_filepath = experience_time + '/position/depth'
recordlidarSem_filepath = experience_time + '/position/lidarSem'
recordLog_filepath = experience_time + '/position/recording.log'

# 相机列表 1代表需要输出该数据 0代表不需要
camera_dic = {
    "rgb": 1,
    "semantic": 0,
    "depth": 0,
    "lidarSem": 0,
}


# 构造相机投影矩阵函数，用于构造相机的投影矩阵，该矩阵用于将三维坐标投影到二维图像上。
def build_projection_matrix(w, h, fov):
    focal = w / (2.0 * np.tan(fov * np.pi / 360.0))  # 计算焦距
    K = np.identity(3)  # 创建一个3x3的单位矩阵 K，该矩阵在相机投影中用于缩放和平移
    K[0, 0] = K[1, 1] = focal  # 将单位矩阵中的对角元素设置为焦距，以进行缩放操作。
    K[0, 2] = w / 2.0  # 将矩阵元素设置为图像的中心点，以便进行平移操作。
    K[1, 2] = h / 2.0
    return K


# 计算三维坐标的二维投影，将车辆的位置映射到图像上。
def get_image_point(loc, c2i, w2c):
    # 格式化输入坐标（loc 是一个 carla.Position 对象）
    point = np.array([loc.x, loc.y, loc.z, 1])

    # 转换到相机坐标系
    point_camera = np.dot(w2c, point)

    # 将坐标系从 UE4 的坐标系转换为标准坐标系（y, -z, x），同时移除第四个分量
    point_camera = [point_camera[1], -point_camera[2], point_camera[0]]

    # 使用相机矩阵进行三维到二维投影
    point_img = np.dot(c2i, point_camera)

    # 归一化
    point_img[0] /= point_img[2]
    point_img[1] /= point_img[2]

    return point_img[0:2]


# 写入车辆位姿信息
def jsonData_write_to_file(file_path, jsonData):
    with open(file_path, 'w') as file:
        file.write(json.dumps(jsonData))


# 设置相机相机文件夹
def set_dir(camera_dic):
    os.makedirs(recordLog_filepath, exist_ok=True)
    if camera_dic["rgb"] == 1:
        os.makedirs(recordImage_filepath, exist_ok=True)

    if camera_dic["semantic"] == 1:
        os.makedirs(recordSemantic_filepath, exist_ok=True)

    if camera_dic["depth"] == 1:
        os.makedirs(recordDepth_filepath, exist_ok=True)

    if camera_dic["lidarSem"] == 1:
        os.makedirs(recordlidarSem_filepath, exist_ok=True)


# 存储rgb
def rgb_save(rgb_image, filename):
    rgb_image.save_to_disk(f'{recordImage_filepath}/{filename}/%08d' % (frame - 50))


cameras_json = {}
frames_data = []

try:
    # 连接Carla并获取世界
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    # 生成车辆
    vehicle_bp = bp_lib.find('vehicle.boxcar.boxcar')
    # spawn_point = random.choice(world.get_map().get_spawn_points())
    # print(spawn_points)
    spawn_point = Transform(Location(x=22.881157, y=-60.899998, z=0.600000),
                            Rotation(pitch=0.000000, yaw=-0.023438, roll=0.000000))
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
    actor_list.append(vehicle)

    # 生成相机
    # 相机文件夹设置
    set_dir(camera_dic)
    # 生成相机
    camera_init_trans = carla.Transform(carla.Location(x=1, y=0, z=2), carla.Rotation(yaw=30, roll=0, pitch=0))
    rgb1_bp = bp_lib.find('sensor.camera.rgb')
    rgb1_bp.set_attribute('image_size_x', '1920')  # 960
    rgb1_bp.set_attribute('image_size_y', '1080')  # 540
    rgb1_camera = world.spawn_actor(rgb1_bp, camera_init_trans, attach_to=vehicle)

    camera_init_trans = carla.Transform(carla.Location(x=1, y=0, z=2), carla.Rotation(yaw=0, roll=0, pitch=0))
    rgb2_bp = bp_lib.find('sensor.camera.rgb')
    rgb2_bp.set_attribute('image_size_x', '1920')  # 960
    rgb2_bp.set_attribute('image_size_y', '1080')  # 540
    rgb2_camera = world.spawn_actor(rgb1_bp, camera_init_trans, attach_to=vehicle)

    camera_init_trans = carla.Transform(carla.Location(x=1, y=0, z=2), carla.Rotation(yaw=-30, roll=0, pitch=0))
    rgb3_bp = bp_lib.find('sensor.camera.rgb')
    rgb3_bp.set_attribute('image_size_x', '1920')  # 960
    rgb3_bp.set_attribute('image_size_y', '1080')  # 540
    rgb3_camera = world.spawn_actor(rgb1_bp, camera_init_trans, attach_to=vehicle)

    # 设置汽车自动行驶
    vehicle.set_autopilot(True)

    # 生成目标车辆
    for i in range(15):
        vehicle_bp = random.choice(bp_lib.filter('vehicle'))
        npc = world.try_spawn_actor(vehicle_bp, random.choice(world.get_map().get_spawn_points()))
        if npc:
            npc.set_autopilot(True)
            actor_list.append(npc)

    # 设置仿真模式为同步模式
    settings = world.get_settings()
    settings.synchronous_mode = True  # 启用同步模式
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    # 创建对接接收相机数据
    rgb1_queue = queue.Queue()
    rgb2_queue = queue.Queue()
    rgb3_queue = queue.Queue()
    rgb1_camera.listen(rgb1_queue.put)
    rgb2_camera.listen(rgb2_queue.put)
    rgb3_camera.listen(rgb3_queue.put)

    # 从相机获取属性
    image_w_rgb1 = int(rgb1_camera.attributes["image_size_x"])  # 图像宽度
    image_h_rgb1 = int(rgb1_camera.attributes["image_size_y"])  # 图像高度
    fov1 = float(rgb1_camera.attributes["fov"])  # 视场角
    # 计算相机投影矩阵，用于从三维坐标投影到二维坐标
    c2i1 = build_projection_matrix(image_w_rgb1, image_h_rgb1, fov1)
    image_w_rgb2 = int(rgb2_camera.attributes["image_size_x"])  # 图像宽度
    image_h_rgb2 = int(rgb2_camera.attributes["image_size_y"])  # 图像高度
    fov2 = float(rgb2_camera.attributes["fov"])  # 视场角
    # 计算相机投影矩阵，用于从三维坐标投影到二维坐标
    c2i2 = build_projection_matrix(image_w_rgb2, image_h_rgb2, fov2)
    image_w_rgb3 = int(rgb3_camera.attributes["image_size_x"])  # 图像宽度
    image_h_rgb3 = int(rgb3_camera.attributes["image_size_y"])  # 图像高度
    fov3 = float(rgb3_camera.attributes["fov"])  # 视场角
    # 计算相机投影矩阵，用于从三维坐标投影到二维坐标
    c2i3 = build_projection_matrix(image_w_rgb3, image_h_rgb1, fov1)
    cameras_json.update({"Width1": image_w_rgb1})
    cameras_json.update({"Height1": image_h_rgb1})
    cameras_json.update({"Fov1": fov1})
    cameras_json.update({"c2i1": c2i1.tolist()})
    cameras_json.update({"Width2": image_w_rgb2})
    cameras_json.update({"Height2": image_h_rgb2})
    cameras_json.update({"Fov2": fov2})
    cameras_json.update({"c2i2": c2i1.tolist()})
    cameras_json.update({"Width3": image_w_rgb3})
    cameras_json.update({"Height3": image_h_rgb3})
    cameras_json.update({"Fov3": fov3})
    cameras_json.update({"c2i3": c2i1.tolist()})

    frame = 1

    # 开始录制
    client.start_recorder(recordLog_filepath)

    # 在 OpenCV 的显示窗口中显示图像
    cv2.namedWindow('rgb1_camera', cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('rgb2_camera', cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow('rgb3_camera', cv2.WINDOW_AUTOSIZE)

    print("actor_list:", actor_list)

    while True:
        # 更新世界状态并获取图像
        world.tick()
        rgb1_image = rgb1_queue.get()
        rgb2_image = rgb2_queue.get()
        rgb3_image = rgb3_queue.get()
        rgb1_img = np.reshape(np.copy(rgb1_image.raw_data), (rgb1_image.height, rgb1_image.width, 4))
        rgb2_img = np.reshape(np.copy(rgb2_image.raw_data), (rgb2_image.height, rgb2_image.width, 4))
        rgb3_img = np.reshape(np.copy(rgb3_image.raw_data), (rgb3_image.height, rgb3_image.width, 4))

        frame_data = {}
        cameras_data = []
        if frame >= 50 and frame <= 150 and frame % 1 == 0:
            camera1_data = {}
            w2c1 = np.array(rgb1_camera.get_transform().get_inverse_matrix())
            # 相机位置
            # 旋转
            pitch = rgb1_camera.get_transform().rotation.pitch
            yaw = rgb1_camera.get_transform().rotation.yaw
            roll = rgb1_camera.get_transform().rotation.roll
            camera_rotation = [yaw, pitch, roll]
            # 位置
            x = rgb1_camera.get_transform().location.x
            y = rgb1_camera.get_transform().location.y
            z = rgb1_camera.get_transform().location.z
            camera_location = [x, y, z]
            camera1_data.update({"camera_rotation": camera_rotation})
            camera1_data.update({"camera_location": camera_location})
            camera1_data.update({"w2c": w2c1.tolist()})
            cameras_data.append(camera1_data)

            camera2_data = {}
            w2c2 = np.array(rgb2_camera.get_transform().get_inverse_matrix())
            # 相机位置
            # 旋转
            pitch = rgb2_camera.get_transform().rotation.pitch
            yaw = rgb2_camera.get_transform().rotation.yaw
            roll = rgb2_camera.get_transform().rotation.roll
            camera_rotation = [yaw, pitch, roll]
            # 位置
            x = rgb2_camera.get_transform().location.x
            y = rgb2_camera.get_transform().location.y
            z = rgb2_camera.get_transform().location.z
            camera_location = [x, y, z]
            camera2_data.update({"camera_rotation": camera_rotation})
            camera2_data.update({"camera_location": camera_location})
            camera2_data.update({"w2c": w2c2.tolist()})
            cameras_data.append(camera2_data)

            camera3_data = {}
            w2c3 = np.array(rgb3_camera.get_transform().get_inverse_matrix())
            # 相机位置
            # 旋转
            pitch = rgb3_camera.get_transform().rotation.pitch
            yaw = rgb3_camera.get_transform().rotation.yaw
            roll = rgb3_camera.get_transform().rotation.roll
            camera_rotation = [yaw, pitch, roll]
            # 位置
            x = rgb3_camera.get_transform().location.x
            y = rgb3_camera.get_transform().location.y
            z = rgb3_camera.get_transform().location.z
            camera_location = [x, y, z]
            camera3_data.update({"camera_rotation": camera_rotation})
            camera3_data.update({"camera_location": camera_location})
            camera3_data.update({"w2c": w2c3.tolist()})
            cameras_data.append(camera3_data)

            frame_data.update({"frame": frame})
            frame_data.update({"cameras_data": cameras_data})
            frames_data.append(frame_data)

            # 存储当前帧的画面图像
            # 存储rgb图像
            rgb_save(rgb1_image, "rgb1")
            rgb_save(rgb2_image, "rgb2")
            rgb_save(rgb3_image, "rgb3")

        cv2.imshow('rgb1_camera', rgb1_img)
        cv2.imshow('rgb2_camera', rgb2_img)
        cv2.imshow('rgb3_camera', rgb3_img)

        if cv2.waitKey(1) == ord('q'):
            break
        frame = frame + 1

    cv2.destroyAllWindows()
    vehicle.destroy()
    rgb1_camera.stop()
    rgb1_camera.destroy()
    rgb2_camera.stop()
    rgb2_camera.destroy()
    rgb3_camera.stop()
    rgb3_camera.destroy()


finally:
    # 存储本次模拟的全部位置信息到jsonData
    cameras_json.update({"framesData": frames_data})

    jsonData_write_to_file(position_filepath, cameras_json)

    for actor in actor_list:
        actor.destroy()
    print("All cleaned up!")
