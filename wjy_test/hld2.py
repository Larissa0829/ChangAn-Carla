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
    vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0, brake=0))
    # vehicle.set_autopilot(True)
    actor_list.append(vehicle)

    # 创建相机蓝图
    camera_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_transform = carla.Transform(carla.Location(x=-5, z=4), carla.Rotation(pitch=-20))
    camera = world.spawn_actor(camera_blueprint, camera_transform, attach_to=vehicle)
    # camera.listen(lambda image: image.save_to_disk(os.path.join('_out', '%06d.png' % image.frame)))

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
                                vehicle.set_target_velocity(carla.Vector3D(1, 0, 0))  # 自行设置合适的速度
                                break

    # 启动红绿灯检测器线程
    traffic_light_thread = threading.Thread(target=process_traffic_lights)
    traffic_light_thread.start()

    # 进入主循环
    while True:
        # 设置相机视角
        world.get_spectator().set_transform(camera.get_transform())

        world.tick()
        # # 设置汽车运动参数
        # vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0, brake=0))
        # # 设置相机监听事件
        # camera.listen(lambda image: image.save_to_disk(os.path.join('_out', '%06d.png' % image.frame)))


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