import carla
import math
import random
import time

# 创建CARLA仿真客户端
client = carla.Client('127.0.0.1', 2000)
client.set_timeout(2.0)
actor_list=[]

try:
    # ======================================获取CARLA世界和地图======================================
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()
    map = world.get_map()

    # ======================================创建两辆车======================================
    # 位置设置
    # spawn_points = map.get_spawn_points()
    spawn_points = carla.Transform(
        carla.Location(x=20.235275, y=13.414804, z=0.600000),  # 设置初始位置的x、y和z坐标
        carla.Rotation(pitch=0.000000, yaw=-179.840790, roll=0.000000)  # 设置初始方向的pitch、yaw和roll角度
    )
    # 主车
    # vehicle_bp1 = blueprint_library.filter('vehicle')[0]
    # vehicle_bp1.set_attribute('color', '255,0,0')
    # 自定义小车
    vehicle_bp1 = blueprint_library.find('vehicle.simplecar.simplecar')
    # vehicle1_spawn_point = random.choice(spawn_points)
    vehicle1_spawn_point = spawn_points
    vehicle1 = world.spawn_actor(vehicle_bp1, vehicle1_spawn_point)
    # 跟车
    # vehicle_bp2 = blueprint_library.filter('vehicle')[0]
    # vehicle_bp2.set_attribute('color', '0,0,255')
    # 自定义小车
    vehicle_bp2 = blueprint_library.find('vehicle.simplecar3.simplecar3')
    vehicle2_spawn_point = carla.Transform(vehicle1_spawn_point.location + carla.Location(x=10.0),carla.Rotation(pitch=0.000000, yaw=-179.840790, roll=0.000000))
    vehicle2 = world.spawn_actor(vehicle_bp2, vehicle2_spawn_point)

    # 设置车辆的初始速度
    vehicle1.apply_control(carla.VehicleControl(throttle=0.5, steer=0, brake=0)) # 车1的初始速度为10 m/s
    vehicle2.apply_control(carla.VehicleControl(throttle=0, steer=0, brake=0))
    actor_list.append(vehicle1)
    actor_list.append(vehicle2)
    # # 给两个车设置自动驾驶
    # vehicle1.set_autopilot(True)
    # vehicle2.set_autopilot(True)


    # ======================================创建相机蓝图======================================
    camera_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_transform = carla.Transform(carla.Location(x=-5, z=4), carla.Rotation(pitch=-10))
    camera1 = world.spawn_actor(camera_blueprint, camera_transform, attach_to=vehicle1)
    camera2 = world.spawn_actor(camera_blueprint, camera_transform, attach_to=vehicle2)
    # camera.listen(lambda image: image.save_to_disk(os.path.join('_out', '%06d.png' % image.frame)))
    actor_list.append(camera1)
    actor_list.append(camera2)

    # ======================================定义跟车参数======================================
    desired_distance = 20.0  # 期望的跟车距离
    max_velocity = 1.0  # 最大速度 (m/s)

    while True:
        # 设置相机视角
        world.get_spectator().set_transform(camera2.get_transform())

        # 获取车辆的位置
        location1 = vehicle1.get_location()
        location2 = vehicle2.get_location()

        # 计算车辆之间的距离
        distance = math.sqrt((location1.x - location2.x)**2 + (location1.y - location2.y)**2)

        # 计算车辆2的期望速度，使其保持在期望跟车距离内
        target_speed = 0.0 if desired_distance > distance else max_velocity

        # 设置车辆2的速度
        vehicle2.apply_control(carla.VehicleControl(throttle=target_speed, steer=0, brake=0))

        # 打印信息
        print(f"Distance between vehicles: {distance:.2f} m, Target Speed for Vehicle 2: {vehicle2.get_velocity().length():.2f} m/s")


        world.tick()

finally:
    client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
    print("All the actors have already been destroied !")
