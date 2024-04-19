import glob
import os
import sys

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

BoundingBox= {(678.5556952464336, 390.8428252573767), (601.4460964425023, 437.10935597184016),
                                    (601.4461474257612, 390.8428263499595), (678.5556267164006, 437.10935078809723),
                                    (687.7629824867607, 455.52362993466875), (592.2391221950392, 398.20830102145885),
                                    (687.762943009126, 398.20829905230335), (592.2389140764958, 455.5235988199856)},
BoundingBox = list(BoundingBox)
print(BoundingBox)

# 生成的对象列表
actor_list = []
# 帧数
frame = 1
# 定义jsonData文件中的信息
jsonData = {}
jsonData = {
    "width": 1280,
    "height": 720,
    "fov": 90.0,
    "c2i": [[640.0000000000001, 0.0, 640.0], [0.0, 640.0000000000001, 360.0], [0.0, 0.0, 1.0]],
    "positionsData": [
        {
            "Frame": 51,
            "carsInfo": [
                {
                    "carId": 71,
                    "Rotation": [-0.6511765122413635, 89.784912109375, -0.005981443915516138],
                    "Location": [-48.56568908691406, 102.70761108398438, 0.0066598509438335896],
                    "Camra_Rotation": [-0.0023837359622120857, 0.25042328238487244, -138.8870391845703],
                    "Camra_Location": [-77.79424285888672, 132.59947204589844, 2.00062894821167],
                    "w2c": [[0.0037537245079874992, 0.99992835521698, -0.011364929378032684, -87.51788330078125],
                            [-0.9999929666519165, 0.0037551536224782467, 0.00010438914614496753, -48.951026916503906],
                            [0.0001470587303629145, 0.011364457197487354, 0.9999353885650635, -3.166735887527466],
                            [0.0, 0.0, 0.0, 1.0]],
                    "c2v": [[1.0, 3.9750420565830825e-19, 2.688711807012403e-21, -0.056007385253913355],
                            [-1.0439958509085464e-19, 0.9999999999999999, 9.775943079590577e-19, -14.976188659667969],
                            [-1.378007291035478e-20, -3.3034574456132107e-19, 1.0, 2.170346870552749],
                            [0.0, 0.0, 0.0, 1.0]],
                    "BoundingBox": {(678.5556952464336, 390.8428252573767), (601.4460964425023, 437.10935597184016),
                                    (601.4461474257612, 390.8428263499595), (678.5556267164006, 437.10935078809723),
                                    (687.7629824867607, 455.52362993466875), (592.2391221950392, 398.20830102145885),
                                    (687.762943009126, 398.20829905230335), (592.2389140764958, 455.5235988199856)},
                },
            ]

        },
    ]

}
# try:
#     # 连接Carla并获取世界
#     client = carla.Client('localhost', 2000)
#     world = client.get_world()
#     bp_lib = world.get_blueprint_library()
#
#     # 生成车辆
#     # vehicle_bp = bp_lib.find('*unikCar2*')
#     # spawn_point = random.choice(world.get_map().get_spawn_points())
#     # print(spawn_points)
#     # spawn_point = Transform(Location(x=-48.567417, y=102.479195, z=0.600000),
#     #                         Rotation(pitch=0.000000, yaw=89.838760, roll=0.000000))
#     # vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
#     vehicle_bp = bp_lib.find('vehicle.audi.a2')
#     spawn_point = Transform(Location(x=-48.567417, y=102.479195, z=0.600000),
#                             Rotation(pitch=0.000000, yaw=89.838760, roll=0.000000))
#     vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
#     actor_list.append(vehicle)
#
#     # 生成相机
#     camera_bp = bp_lib.find('sensor.camera.rgb')
#     camera_bp.set_attribute('image_size_x', '1280')  # 960
#     camera_bp.set_attribute('image_size_y', '720')  # 540
#     camera_init_trans = carla.Transform(carla.Location(x=-15, y=0, z=2), carla.Rotation(yaw=0, roll=0, pitch=0))
#     camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)
#     actor_list.append(camera)
#
#     # 设置仿真模式为同步模式
#     settings = world.get_settings()
#     settings.synchronous_mode = True  # 启用同步模式
#     settings.fixed_delta_seconds = 0.05
#     world.apply_settings(settings)
#
#     # 创建对接接收相机数据
#     image_queue = queue.Queue()
#     camera.listen(image_queue.put)
#
#     # 获取第一张图像（这里相当于是frame=0）
#     world.tick()
#     image = image_queue.get()
#
#     # 将原始数据重新整形为 RGB 数组
#     img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
#
#     # 在 OpenCV 的显示窗口中显示图像
#     cv2.namedWindow('ImageWindowName', cv2.WINDOW_AUTOSIZE)
#     cv2.imshow('ImageWindowName', img)
#     cv2.waitKey(1)
#
#     while True:
#         # 更新世界状态并获取图像
#         world.tick()
#         image = image_queue.get()
#
#         img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
#
#         if frame >= 50 and frame % 3 == 0:  # 50帧之后且每三帧取一次
#             pass
#
#         cv2.imshow('ImageWindowName', img)
#         if cv2.waitKey(1) == ord('q'):
#             break
#         frame = frame + 1
#
#     cv2.destroyAllWindows()
#     vehicle.destroy()
#     camera.stop()
#     camera.destroy()
#
# except:
#     print(Exception)
#
# finally:
#     for actor in actor_list:
#         actor.destroy()
