import glob
import os
import sys

'''
生产所有完整的数据

要多少个相机：CAMERA_COUNT = 2
每个相机的方向：DIRECTIONS = [
    [1, 0, 2, 0, 0, 0],
    [1, 0, 2, CAMERA_ANGLE, 0, 0],
    # [1, 0, 2, -CAMERA_ANGLE, 0, 0],
]
设置需要的相机数据类型：camera_dic = {
    "rgb": CAMERA_COUNT,
    "semantic_segmentation": CAMERA_COUNT,
    "ray_cast_semantic": CAMERA_COUNT,
} # key的命名按照carla官方相机类型命名

输出数据结构：
每个相机类型文件夹里面有相机个数个文件夹，里面存放每个相机类型输出的数据
position{'CAMERA_COUNT'}.json文件中存放每个rgb相机中数据。
数据结构：
{
  "width": 1280,
  "height": 960,
  "fov": 90.0,
  "c2i": [
    [
      640.0000000000001,
      0.0,
      640.0
    ],
    [
      0.0,
      640.0000000000001,
      480.0
    ],
    [
      0.0,
      0.0,
      1.0
    ]
  ],
  "framesData": [
    {
      "frame": 51,
      "myCarInfo": [
        {
          "carLocation": [
            23.000612258911133,
            -60.90010070800781,
            0.004736633040010929
          ],
          "carRotation": [
            -0.026580810546875,
            -0.46801820397377014,
            -0.000518798828125
          ],
          "camerasPosition": {
            "cameraRotation": [
              -0.026580810546875,
              -0.46801820397377014,
              -0.000518798828125
            ],
            "cameraLocation": [
              24.016916275024414,
              -60.90058898925781,
              1.9965015649795532
            ],
            "w2c": [
              [
                0.9999665021896362,
                -0.000463907141238451,
                -0.00816836766898632,
                -24.02805519104004
              ],
              [
                0.00046399657730944455,
                0.9999998807907104,
                9.05444539966993e-06,
                60.88941955566406
              ],
              [
                0.008168362081050873,
                -1.284423706238158e-05,
                0.9999666213989258,
                -2.1933960914611816
              ],
              [
                0.0,
                0.0,
                0.0,
                1.0
              ]
            ],
            "c2v": [
              [
                -0.004124122153903104,
                -0.9999603294253584,
                0.007897812324638433,
                85.02732190579158
              ],
              [
                0.999991505276179,
                -0.004123560183269461,
                8.741613921824613e-05,
                -166.64119577598422
              ],
              [
                -5.484555519217665e-05,
                0.007898106418727475,
                0.9999688090587918,
                1.527776835856601
              ],
              [
                0.0,
                0.0,
                0.0,
                1.0
              ]
            ]
          }
        }
      ],
      "carsInfo": [
        {
          "carId": 26,
          "Rotation": [
            -167.12010192871094,
            0.4924907684326172,
            1.8521196842193604
          ],
          "Location": [
            -68.73882293701172,
            129.3050537109375,
            -0.07526300102472305
          ],
          "w2c": [
            [
              0.0028141262009739876,
              0.9999960660934448,
              0.0,
              -103.34212493896484
            ],
            [
              -0.9999960660934448,
              0.0028141262009739876,
              -0.0,
              -48.855613708496094
            ],
            [
              -0.0,
              0.0,
              1.0,
              -2.0000014305114746
            ],
            [
              0.0,
              0.0,
              0.0,
              1.0
            ]
          ],
          "c2v": [
            [
              -0.2252632307714345,
              0.9737609661970014,
              0.03234282859859015,
              -189.9587110497859
            ],
            [
              -0.9741733882851292,
              -0.22564111428503794,
              0.008504496809996888,
              65.69283906980776
            ],
            [
              0.015579217739474814,
              -0.029591768258869,
              0.9994406918647901,
              6.9724846101330265
            ],
            [
              0.0,
              0.0,
              0.0,
              1.0
            ]
          ],
          "BoundingBox": [
            [
              713.3670666628356,
              322.0241617872172
            ],
            [
              713.4376923877464,
              327.78384222811474
            ],
            [
              1202.4455068451714,
              374.70093960836897
            ],
            [
              1089.7720353508,
              406.9608427443012
            ],
            [
              1089.730373381451,
              375.1520930341491
            ],
            [
              1070.0962845038923,
              403.9372031850047
            ],
            [
              1232.6412457424324,
              375.72791327806584
            ],
            [
              1202.4918044289316,
              405.66685819658676
            ],
            [
              1232.6927960162018,
              408.9510142226733
            ],
            [
              1070.0584914396065,
              374.2035457406783
            ]
          ]
        }
      ]
    }
  ]
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

# 全局值
VIDIDIS = np.array(cm.get_cmap("plasma").colors)
VID_RANGE = np.linspace(0.0, 1.0, VIDIDIS.shape[0])
CAMERA_WIDTH = 1280
CAMERA_HEIGHT = 960
CAMERA_ANGLE = 30
CAMERA_FOV = 90
CAMERA_COUNT = 2
DIRECTIONS = [
    [1, 0, 2, 0, 0, 0],
    [1, 0, 2, CAMERA_ANGLE, 0, 0],
    # [1, 0, 2, -CAMERA_ANGLE, 0, 0],
]
FRAME = 1

# 生成的对象列表
actor_list = []
# 位姿输出文件路径
experience_time = "2/position/"  # 实验次数，也可以代表场景次数

record_filepath = {}
recordLog_filepath = experience_time + 'recording.log'

# 所有rgb相机中的camerasData
camerasData = []

# 相机列表 CAMERA_COUNT代表有几个摄像机需要输出数据 0代表不需要
camera_dic = {
    "rgb": CAMERA_COUNT,
    "semantic_segmentation": CAMERA_COUNT,
    "ray_cast_semantic": CAMERA_COUNT,
}


# 设置相机相机文件夹
def set_dir():
    os.makedirs(recordLog_filepath, exist_ok=True)
    for key in camera_dic.keys():
        record_filepath[key] = experience_time + key
        os.makedirs(record_filepath[key], exist_ok=True)


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
def cameraData_write_to_file(file_path, cameraData):
    with open(file_path, 'w') as file:
        file.write(json.dumps(cameraData))


# 判断bbox点位是否合理
def is_valid_point(p):
    image_w = CAMERA_WIDTH
    image_h = CAMERA_HEIGHT
    return not np.isnan(p[0]) and not np.isnan(p[1]) and p[0] >= 0 and p[0] < image_w and p[1] >= 0 and p[1] < image_h


# 生成相机
def generate_camera(world, bp_lib, directions):
    cameras = {}
    queues = {}
    for i in range(0, CAMERA_COUNT):
        camerasData.append({})
    for key in camera_dic.keys():
        cameras[key] = []
        queues[key] = []
        for i in range(0, CAMERA_COUNT):
            x, y, z, yaw, roll, pitch = directions[i]
            camera_init_trans = carla.Transform(carla.Location(x=x, y=y, z=z),
                                                carla.Rotation(yaw=yaw, roll=roll, pitch=pitch))

            if key == "ray_cast_semantic":
                delta = 0.05
                bp = bp_lib.find(f'sensor.lidar.{key}')
                bp.set_attribute("upper_fov", str(15.0))
                bp.set_attribute("lower_fov", str(-25.0))
                bp.set_attribute("channels", str(64.0))
                bp.set_attribute("range", str(100.0))
                bp.set_attribute("rotation_frequency", str(1.0 / delta))
                bp.set_attribute("points_per_second", str(500000))
            else:
                bp = bp_lib.find(f'sensor.camera.{key}')
                bp.set_attribute('image_size_x', f'{CAMERA_WIDTH}')
                bp.set_attribute('image_size_y', f'{CAMERA_HEIGHT}')
            camera = world.spawn_actor(bp, camera_init_trans, attach_to=vehicle)
            # 创建对接接收相机数据
            image_queue = queue.Queue()
            camera.listen(image_queue.put)

            queues[key].append(image_queue)
            cameras[key].append(camera)

    return [cameras, queues]


def save_cameras_data(key, que):
    if key == "semantic_segmentation":
        semantic_image = que.get()
        semantic_image.convert(carla.ColorConverter.CityScapesPalette)
        semantic_image.save_to_disk(f'{record_filepath[key]}/{i}/%08d' % FRAME)

    elif key == "ray_cast_semantic":
        lidarSem_ply = que.get()
        point_list = o3d.geometry.PointCloud()

        # We need to convert point cloud(carla-format) into numpy.ndarray
        data = np.copy(np.frombuffer(lidarSem_ply.raw_data, dtype=np.dtype("f4")))
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
        # point_list.colors = o3d.utility.Vector3dVector(int_color)
        # 保存PointCloud对象到PLY文件
        os.makedirs(f"{record_filepath[key]}/{i}", exist_ok=True)
        o3d.io.write_point_cloud(f"{record_filepath[key]}/{i}/%08d.ply" % FRAME, point_list)


# 存储自车位置信息
def mycarPosition_save(rgb_camera):
    # 车辆位置
    # 旋转
    pitch = vehicle.get_transform().rotation.pitch
    yaw = vehicle.get_transform().rotation.yaw
    roll = vehicle.get_transform().rotation.roll
    rotation = [yaw, pitch, roll]
    # 位置
    x = vehicle.get_transform().location.x
    y = vehicle.get_transform().location.y
    z = vehicle.get_transform().location.z
    location = [x, y, z]

    # 相机位置
    camera_position = {}
    # 旋转
    pitch = rgb_camera.get_transform().rotation.pitch
    yaw = rgb_camera.get_transform().rotation.yaw
    roll = rgb_camera.get_transform().rotation.roll
    camera_rotation = [yaw, pitch, roll]
    # 位置
    x = rgb_camera.get_transform().location.x
    y = rgb_camera.get_transform().location.y
    z = rgb_camera.get_transform().location.z
    camera_location = [x, y, z]
    # 计算相机到npc汽车的旋转矩阵
    # 定义获取相机投影矩阵
    w2c = np.array(rgb_camera.get_transform().get_inverse_matrix())
    c2w = np.array(rgb_camera.get_transform().get_matrix())
    w2v = np.linalg.inv(np.array(npc.get_transform().get_matrix()))
    c2v = np.dot(c2w, w2v)

    # 存储每一帧每一辆车的每一个信息
    camera_position.update({"cameraRotation": camera_rotation})
    camera_position.update({"cameraLocation": camera_location})
    camera_position.update({"w2c": w2c.tolist()})
    camera_position.update({"c2v": c2v.tolist()})

    mycar = {}
    mycar.update(({"carLocation": location}))
    mycar.update(({"carRotation": rotation}))
    mycar.update(({"camerasPosition": camera_position}))
    mycar_info.append(mycar)


def othercarsPosition_save(rgb_camera):
    w2c = np.array(rgb_camera.get_transform().get_inverse_matrix())

    # 绘画bbox并存储
    for npc in world.get_actors().filter('*vehicle*'):
        if npc.id != vehicle.id:
            dist = npc.get_transform().location.distance(rgb_camera.get_transform().location)
            # 筛选距离在50米以内的车辆
            if dist < 50:
                # 定义每辆车的信息
                carInfo = {}
                bb = npc.bounding_box
                # 定义所有车辆边界框信息列表
                bounding_box_points_list = []
                forward_vec = vehicle.get_transform().get_forward_vector()
                ray = npc.get_transform().location - vehicle.get_transform().location
                distance = forward_vec.dot(ray)
                # 计算车辆前进方向与车辆之间的向量的点积，
                # 通过阈值判断是否在相机前方绘制边界框
                if distance >= 0:
                    # 车辆位置
                    # 旋转
                    pitch = npc.get_transform().rotation.pitch
                    yaw = npc.get_transform().rotation.yaw
                    roll = npc.get_transform().rotation.roll
                    rotation = [yaw, pitch, roll]
                    # 位置
                    x = npc.get_transform().location.x
                    y = npc.get_transform().location.y
                    z = npc.get_transform().location.z
                    location = [x, y, z]

                    # 计算相机到npc汽车的旋转矩阵
                    c2w = np.array(rgb_camera.get_transform().get_matrix())
                    w2v = np.linalg.inv(np.array(npc.get_transform().get_matrix()))
                    c2v = np.dot(c2w, w2v)

                    # 距离
                    verts = [v for v in bb.get_world_vertices(npc.get_transform())]
                    # 定义单个车辆的边界框点位信息数组
                    bounding_box_points = set()  # 存放八个点

                    for edge in edges:
                        p1 = get_image_point(verts[edge[0]], c2i, w2c)
                        p2 = get_image_point(verts[edge[1]], c2i, w2c)
                        if is_valid_point(p1) and is_valid_point(p2):
                            bounding_box_points.add(tuple(p1))
                            bounding_box_points.add(tuple(p2))
                            # cv2.line(img, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), (255, 0, 0, 255),
                            #          1)
                    # 这里可以取消注释看boundingbox的结果
                    # cv2.imwrite(f'{FRAME}.png', img)

                    for point in bounding_box_points:
                        point = list(point)
                        bounding_box_points_list.append(point)

                    # 存储每一帧每一辆车的每一个信息
                    carInfo.update({"carId": npc.id})
                    carInfo.update({"rotation": rotation})
                    carInfo.update({"location": location})
                    carInfo.update({"w2c": w2c.tolist()})
                    carInfo.update({"c2v": c2v.tolist()})
                    carInfo.update({"boundingBox": bounding_box_points_list})

                    carsInfo.append(carInfo)


try:
    # 连接Carla并获取世界
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    # 生成车辆
    vehicle_bp = bp_lib.find('vehicle.boxcar.boxcar')
    # spawn_point = random.choice(world.get_map().get_spawn_points())
    # print(spawn_points)Transform(Location(x=22.881157, y=-60.899998, z=0.600000), Rotation(pitch=0.000000, yaw=-0.023438, roll=0.000000))
    spawn_point = Transform(Location(x=22.881157, y=-60.899998, z=0.600000),
                            Rotation(pitch=0.000000, yaw=-0.023438, roll=0.000000))
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
    actor_list.append(vehicle)
    # 设置汽车自动行驶
    vehicle.set_autopilot(True)

    # 生成相机
    # 相机文件夹设置
    set_dir()
    # 生成相机 cameras={"rgb":[rgb1,rgb2……],"semantic_segmentation":[……]，……}

    cameras, queues = generate_camera(world, bp_lib, directions=DIRECTIONS)
    # for camera in cameras:
    #     if camera is not None:
    #         actor_list.append(camera)

    # 生成目标车辆
    for i in range(5):
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

    # 从相机获取属性
    for key in camera_dic.keys():
        if key == "rgb":
            for i in range(0, CAMERA_COUNT):
                image_w_rgb = int(cameras[key][i].attributes["image_size_x"])
                image_h_rgb = int(cameras[key][i].attributes["image_size_y"])
                fov = float(cameras[key][i].attributes["fov"])
                c2i = build_projection_matrix(image_w_rgb, image_h_rgb, fov)

                camerasData[i].update({"width": image_w_rgb})
                camerasData[i].update({"height": image_h_rgb})
                camerasData[i].update({"fov": fov})
                camerasData[i].update({"c2i": c2i.tolist()})

    # 定义要检测的物体集合
    bounding_box_set = world.get_level_bbs(carla.CityObjectLabel.Car)
    # 定义boundingbox绘制顶点顺序
    edges = [[0, 1], [1, 3], [3, 2], [2, 0], [0, 4], [4, 5], [5, 1], [5, 7], [7, 6], [6, 4], [6, 2], [7, 3]]

    # 开始录制
    client.start_recorder(recordLog_filepath)

    # 在 OpenCV 的显示窗口中显示图像
    cv2.namedWindow('ImageWindowName', cv2.WINDOW_AUTOSIZE)

    # 每一帧需要存储的信息
    frames_data = []
    while True:
        # 更新世界状态并获取图像
        world.tick()
        rgb_images = []
        rgb_show_images = []
        que = {}
        for i in range(0, CAMERA_COUNT):
            for key in camera_dic.keys():
                if key == "rgb":
                    rgb_image = queues["rgb"][i].get()
                    rgb_img = np.reshape(np.copy(rgb_image.raw_data), (rgb_image.height, rgb_image.width, 4))
                    rgb_images.append(rgb_image)
                    rgb_show_images.append(rgb_img)
                else:
                    if key not in que:
                        que.update({key: [queues[key][i].get()]})
                    else:
                        que[key].append(queues[key][i].get())

        if FRAME >= 50 and FRAME % 10 == 1:
            for i in range(0, CAMERA_COUNT):
                frames_data.append([])
                camera_info = []
                # 每一帧自车位置信息
                mycar_info = []
                # 每一帧中每辆车的信息
                carsInfo = []

                # 定义每一帧的数据对象
                frame_data = {}

                # 先存储一遍自车的位置信息到每一个相机文件中
                mycarPosition_save(cameras["rgb"][i])
                frame_data.update({"frame": FRAME})
                frame_data.update({"myCarInfo": mycar_info})

                othercarsPosition_save(cameras["rgb"][i])

                # 存储每一帧的全部车辆信息到frame_data
                frame_data.update({"carsInfo": carsInfo})
                # 存储每一帧的全部信息到frames_data
                frames_data[i].append(frame_data)
                camerasData[i].update({"framesData": frames_data[i]})

                # 存储当前帧的画面图像
                for j in range(0, len(camera_dic)):
                    for key in camera_dic.keys():
                        if key == "rgb":
                            # 存储rgb图像
                            rgb_images[i].save_to_disk(f'{record_filepath["rgb"]}/{i}/%08d' % FRAME)
                        else:
                            # save_cameras_data(key, queues[key][i])
                            if key == "semantic_segmentation":
                                semantic_image = que[key][i]
                                semantic_image.convert(carla.ColorConverter.CityScapesPalette)
                                semantic_image.save_to_disk(f'{record_filepath[key]}/{i}/%08d' % FRAME)

                            elif key == "ray_cast_semantic":
                                lidarSem_ply = que[key][i]
                                point_list = o3d.geometry.PointCloud()
                                # We need to convert point cloud(carla-format) into numpy.ndarray
                                data = np.copy(np.frombuffer(lidarSem_ply.raw_data, dtype=np.dtype("f4")))
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
                                # point_list.colors = o3d.utility.Vector3dVector(int_color)
                                # 保存PointCloud对象到PLY文件
                                os.makedirs(f"{record_filepath[key]}/{i}", exist_ok=True)
                                o3d.io.write_point_cloud(f"{record_filepath[key]}/{i}/%08d.ply" % FRAME, point_list)

        cv2.imshow('ImageWindowName', rgb_show_images[0])
        if cv2.waitKey(1) == ord('q'):
            break
        FRAME = FRAME + 1

    cv2.destroyAllWindows()
    vehicle.destroy()

    for key in camera_dic.keys():
        for i in range(0, CAMERA_COUNT):
            cameras[key][i].stop()
            cameras[key][i].destroy()

    # 信息写入文件
    for i in range(0, CAMERA_COUNT):
        # 存储本次模拟的全部位置信息到cameraData
        if len(camerasData[i]) != 0:
            # 写入文件
            position_filepath = f'{experience_time}position{i}.json'
            cameraData_write_to_file(position_filepath, camerasData[i])


finally:
    for actor in actor_list:
        actor.destroy()

    print("All cleaned up!")
