import glob
import os
import sys
'''
本代码针对场景：
输入：主车头上有个摄像头，和场景中的车辆
输出：每几帧存储当前摄像机中的画面（可显示boundingbox），相机拍摄到的场景中每一帧、每辆车的数据
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

# 生成的对象列表
actor_list = []
# 位姿输出文件路径
experience_time = "1"  # 实验次数，也可以代表场景次数
position_filepath = experience_time + '/position/position.json'
recordImage_filepath = experience_time + '/position/images'
recordLog_filepath = experience_time + '/position/recording.log'

# 创建文件夹
os.makedirs(recordImage_filepath, exist_ok=True)

# 所有的jsonData
jsonData = {}
# 每一帧需要存储的信息
positionsData = []


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


# 去掉异常的点
def is_valid_point(p):
    return not np.isnan(p[0]) and not np.isnan(p[1]) and p[0] >= 0 and p[0] < image_w and p[1] >= 0 and p[1] < image_h


try:
    # 连接Carla并获取世界
    client = carla.Client('localhost', 2000)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()

    # 生成车辆
    vehicle_bp = bp_lib.find('vehicle.boxcar.boxcar')
    # spawn_point = random.choice(world.get_map().get_spawn_points())
    # print(spawn_points)
    spawn_point = Transform(Location(x=-48.567417, y=102.479195, z=0.006659),
                            Rotation(pitch=0.000000, yaw=89.838760, roll=0.000000))
    vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
    actor_list.append(vehicle)

    # 生成相机
    camera_bp = bp_lib.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '1280')  # 960
    camera_bp.set_attribute('image_size_y', '720')  # 540
    camera_init_trans = carla.Transform(carla.Location(x=1, y=0, z=2), carla.Rotation(yaw=0, roll=0, pitch=0))
    camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)
    actor_list.append(camera)
    vehicle.set_autopilot(True)

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

    # 创建对接接收相机数据
    image_queue = queue.Queue()
    camera.listen(image_queue.put)

    # 从相机获取属性
    image_w = camera_bp.get_attribute("image_size_x").as_int()  # 图像宽度
    image_h = camera_bp.get_attribute("image_size_y").as_int()  # 图像高度
    fov = camera_bp.get_attribute("fov").as_float()  # 视场角
    # 计算相机投影矩阵，用于从三维坐标投影到二维坐标
    c2i = build_projection_matrix(image_w, image_h, fov)

    jsonData.update({"width": image_w})
    jsonData.update({"height": image_h})
    jsonData.update({"fov": fov})
    jsonData.update({"c2i": c2i.tolist()})

    # 定义要检测的物体集合
    bounding_box_set = world.get_level_bbs(carla.CityObjectLabel.Car)
    # 定义boundingbox绘制顶点顺序
    edges = [[0, 1], [1, 3], [3, 2], [2, 0], [0, 4], [4, 5], [5, 1], [5, 7], [7, 6], [6, 4], [6, 2], [7, 3]]
    # 定义帧数
    frame = 1

    # 开始录制
    client.start_recorder(recordLog_filepath)

    # 获取第一张图像（这里相当于是frame=0）
    world.tick()
    image = image_queue.get()
    # 将原始数据重新整形为 RGB 数组
    img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

    # 在 OpenCV 的显示窗口中显示图像
    cv2.namedWindow('ImageWindowName', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('ImageWindowName', img)
    cv2.waitKey(1)

    while True:
        # 更新世界状态并获取图像
        world.tick()
        image = image_queue.get()
        img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

        if frame >= 1 and frame % 10 == 1:  # 50帧之后且每三帧取一次
            # 定义每一帧的数据对象
            positionData = {}
            # 每一帧中每辆车的信息
            carsInfo = []
            # 定义所有车辆边界框信息列表
            bounding_box_points_list = []
            # 定义旋转角存储列表
            rotation = []
            # 定义世界坐标存储列表
            location = []
            # 定义获取相机投影矩阵
            w2c = np.array(camera.get_transform().get_inverse_matrix())
            # 定义c2v初始化
            c2v = np.eye(4)

            # 检测车辆边界框
            for npc in world.get_actors().filter('*vehicle*'):
                if npc.id != vehicle.id:
                    # 定义每辆车的信息
                    carInfo = {}
                    bb = npc.bounding_box
                    dist = npc.get_transform().location.distance(camera.get_transform().location)

                    # 筛选距离在50米以内的车辆
                    if dist < 100:
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

                            # 相机位置
                            # 旋转
                            pitch = camera.get_transform().rotation.pitch
                            yaw = camera.get_transform().rotation.yaw
                            roll = camera.get_transform().rotation.roll
                            camera_rotation = [yaw, pitch, roll]
                            # 位置
                            x = camera.get_transform().location.x
                            y = camera.get_transform().location.y
                            z = camera.get_transform().location.z
                            camera_location = [x, y, z]

                            # 计算相机到npc汽车的旋转矩阵
                            c2w = np.array(camera.get_transform().get_matrix())
                            w2v = np.linalg.inv(np.array(npc.get_transform().get_matrix()))
                            c2v = np.dot(c2w, w2v)

                            # 距离
                            p1 = get_image_point(bb.location, c2i, w2c)
                            verts = [v for v in bb.get_world_vertices(npc.get_transform())]
                            # 定义单个车辆的边界框点位信息数组
                            bounding_box_points = set()  # 存放八个点

                            for edge in edges:
                                p1 = get_image_point(verts[edge[0]], c2i, w2c)

                                p2 = get_image_point(verts[edge[1]], c2i, w2c)

                                # Check if both points are valid
                                if is_valid_point(p1) and is_valid_point(p2):
                                    bounding_box_points.add(tuple(p1))
                                    bounding_box_points.add(tuple(p2))
                                    cv2.line(img, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), (255, 0, 0, 255),
                                             1)
                            # cv2.imwrite(f'{frame}.png', img)

                            for point in bounding_box_points:
                                point = list(point)
                                bounding_box_points_list.append(point)

                        # 存储每一帧每一辆车的每一个信息
                        carInfo.update({"carId": npc.id})
                        carInfo.update({"Rotation": rotation})
                        carInfo.update({"Location": location})
                        carInfo.update({"Camra_Rotation": camera_rotation})
                        carInfo.update({"Camra_Location": camera_location})
                        carInfo.update({"w2c": w2c.tolist()})
                        carInfo.update({"c2v": c2v.tolist()})
                        carInfo.update({"BoundingBox": bounding_box_points_list})

                        carsInfo.append(carInfo)

            # 存储每一帧的全部车辆信息到positionData
            if len(carsInfo) != 0:
                positionData.update({"Frame": frame})
                positionData.update({"carsInfo": carsInfo})
                # 存储每一帧的全部信息到positionsData
                positionsData.append(positionData)

            # 存储当前帧的画面图像
            image.save_to_disk(f'{recordImage_filepath}/%08d' % frame)

        cv2.imshow('ImageWindowName', img)
        if cv2.waitKey(1) == ord('q'):
            break
        frame = frame + 1

    cv2.destroyAllWindows()
    vehicle.destroy()
    camera.stop()
    camera.destroy()


finally:
    # 存储本次模拟的全部位置信息到jsonData
    if len(positionsData) != 0:
        jsonData.update({"positionsData": positionsData})
    # 写入文件
    jsonData_write_to_file(position_filepath, jsonData)

    for actor in actor_list:
        actor.destroy()
    print("All cleaned up!")

