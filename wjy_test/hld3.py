import math
import os
import threading

import carla
import random
import time

# 定义CARLA的主机和端口
host = "127.0.0.1"
port = 2000

# 创建CARLA客户端
client = carla.Client(host, port)
client.set_timeout(2.0)
actor_list=[]

try:
    # 获取CARLA世界
    world = client.get_world()

    # 获取地图
    world_map = world.get_map()


    # 创建车辆蓝图
    vehicle_blueprint = random.choice(world.get_blueprint_library().filter('vehicle.*'))
    transform = random.choice(world.get_map().get_spawn_points())
    # spawn the car here
    vehicle = world.spawn_actor(vehicle_blueprint, transform)
    vehicle.apply_control(carla.VehicleControl(throttle=10.0, steer=0, brake=0))
    # vehicle.set_autopilot(True)
    actor_list.append(vehicle)

    # 创建相机蓝图
    camera_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_transform = carla.Transform(carla.Location(x=-5, z=4), carla.Rotation(pitch=-20))
    camera = world.spawn_actor(camera_blueprint, camera_transform, attach_to=vehicle)
    # camera.listen(lambda image: image.save_to_disk(os.path.join('_out', '%06d.png' % image.frame)))

    actor_list.append(camera)

    # 进入主循环
    while True:
        # 设置相机视角
        world.get_spectator().set_transform(camera.get_transform())

        # 获取车辆的位置和朝向
        vehicle_location = vehicle.get_location()
        vehicle_rotation = vehicle.get_transform().rotation

        # 获取车辆正面的向量
        forward_vector = carla.Vector3D(math.cos(math.radians(vehicle_rotation.yaw)),
                                        math.sin(math.radians(vehicle_rotation.yaw)), 0)

        # 获取车辆正面的区域
        forward_area = vehicle_location + carla.Location(forward_vector * 20)

        # 获取车辆右侧的区域
        right_vector = carla.Vector3D(-forward_vector.y, forward_vector.x, 0)
        right_area = vehicle_location + carla.Location(right_vector * 10)

        # 获取交通信号灯
        traffic_lights = world.get_actors().filter('traffic.traffic_light')

        # 仅保留车辆正面右侧的红绿灯
        forward_right_lights = []
        for traffic_light in traffic_lights:
            light_location = traffic_light.get_location()

            # 计算车辆与红绿灯的向量
            light_vector = light_location - vehicle_location
            light_vector = carla.Vector3D(light_vector.x, light_vector.y, 0)

            # 计算车辆与红绿灯的夹角
            angle = math.degrees(
                math.acos(light_vector.dot(forward_vector) / (light_vector.length() * forward_vector.length())))

            # 计算红绿灯与车辆右侧的位置关系
            right_distance = light_vector.dot(right_vector)

            # 只保留夹角在90度以内且红绿灯在车辆右侧的红绿灯
            if angle <= 90 and right_distance > 0:
                forward_right_lights.append(traffic_light)

        nearest_light = None
        nearest_distance = float('inf')
        for light in forward_right_lights:
            light_location = light.get_location()
            distance = light_location.distance(vehicle_location)
            if distance < nearest_distance:
                nearest_light = light
                nearest_distance = distance

        if nearest_light is not None:
            # 获取信号灯状态
            traffic_light_state = nearest_light.get_state()

            if traffic_light_state == carla.TrafficLightState.Red:
                # 红灯，停车等待
                vehicle.apply_control(carla.VehicleControl(throttle=0, brake=1.0))
            else:
                # 其他情况，正常行驶
                vehicle.apply_control(carla.VehicleControl(throttle=1.0, brake=0))
        else:
            # 没有找到满足条件的红绿灯，正常行驶
            vehicle.apply_control(carla.VehicleControl(throttle=1.0, brake=0))

        # 优化循环性能
        world.tick()

finally:
    # 销毁所有CARLA角色
    client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
'''
import os
import threading

import carla
import random
import time

# 定义CARLA的主机和端口
host = "127.0.0.1"
port = 2000

# 创建CARLA客户端
client = carla.Client(host, port)
client.set_timeout(2.0)
actor_list=[]

try:
    # 获取CARLA世界
    world = client.get_world()

    # 获取地图
    world_map = world.get_map()

    # 创建车辆蓝图
    # spawn_point = world.get_map().get_spawn_points()
    # random.shuffle(spawn_point)
    # spawn_point = random.choice(spawn_point) if spawn_point else carla.Transform()
    # vehicle_blueprint = random.choice(world.get_blueprint_library().filter('vehicle.*'))
    # vehicle_blueprint.set_attribute('role_name', 'WJY001')
    # vehicle = world.spawn_actor(vehicle_blueprint,spawn_point)
    vehicle_blueprint = random.choice(world.get_blueprint_library().filter('vehicle.*'))
    vehicle = world.spawn_actor(vehicle_blueprint,
                                carla.Transform(carla.Location(x=230, y=198, z=40), carla.Rotation()))
    actor_list.append(vehicle)
    vehicle.set_autopilot(True)


    # 创建相机蓝图
    camera_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_blueprint.set_attribute('image_size_x','480')
    camera_blueprint.set_attribute('image_size_y','640')
    camera_blueprint.set_attribute('fov','110')
    camera_blueprint.set_attribute('sensor_tick','1.0')
    camera_transform = carla.Transform(carla.Location(x=-5.5, z=2.8), carla.Rotation(pitch=-15))
    camera = world.spawn_actor(camera_blueprint, camera_transform, attach_to=vehicle)
    camera.listen(lambda image: image.save_to_disk(os.path.join('_out', '%06d.png' % image.frame)))
    actor_list.append(camera)

    # 创建碰撞传感器蓝图
    collision_blueprint = world.get_blueprint_library().find('sensor.other.collision')
    collision_sensor = world.spawn_actor(collision_blueprint, carla.Transform(), attach_to=vehicle)
    actor_list.append(collision_sensor)

    # 创建红绿灯检测器
    traffic_lights = world.get_actors().filter('traffic.traffic_light')

    def process_traffic_lights():
        while True:
            for traffic_light in traffic_lights:
                if vehicle.is_at_traffic_light():
                    traffic_light_state = traffic_light.get_state()
                    if traffic_light_state == carla.TrafficLightState.Red:
                        # 红灯时停下
                        vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
                    elif traffic_light_state == carla.TrafficLightState.Green:
                        # 绿灯时检测斑马线并减速
                        for waypoint in world_map.get_waypoints(vehicle.get_location()):
                            if waypoint.is_crosswalk:
                                vehicle.set_target_velocity(carla.Vector3D(10, 0, 0))  # 自行设置合适的速度
                                break

    # 启动红绿灯检测器线程
    traffic_light_thread = threading.Thread(target=process_traffic_lights)
    traffic_light_thread.start()

    # 设置汽车运动参数
    vehicle.apply_control(carla.VehicleControl(throttle=1.0,steer=0,brake=0))
    # 设置相机监听事件
    camera.listen(lambda image: image.save_to_disk(os.path.join('_out', '%06d.png' % image.frame)))

    # 进入主循环
    while True:
        world.tick()

finally:
    # 销毁所有CARLA角色
    client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
'''