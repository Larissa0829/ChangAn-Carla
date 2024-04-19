# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================
import glob
import math
import os
import random
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


def hld(world, vehicle, vehicle_max_speed):
    global vehicle_current_location
    global way_near_point
    global distance_to_intersection
    # if vehicle:
    #     # 获取汽车当前位置
    #     vehicle_current_location = vehicle.get_location()
    #     # 使用Carla的道路地图获取最近的路口（交叉口）Waypoint
    #     waypoint_location = world.get_map().get_waypoint(vehicle_current_location).transform.location
    #     # 使用Waypoint的distance属性获取距离路口的距离
    #     print(vehicle_current_location)
    #     print(waypoint_location)
    #     distance_to_intersection = math.sqrt((vehicle_current_location.x - waypoint_location.x) ** 2 + (
    #                 vehicle_current_location.y - waypoint_location.y) ** 2)
    #     print("The neaerest way point's distance is：", distance_to_intersection, "meters")
    # else:
    #     print("_________________Not found the vehicle !_________________")
    #     return

    # 获取汽车的速度
    vehicle_current_velocity = vehicle.get_velocity()
    vehicle_current_speed = vehicle_current_velocity.length()  # 线速度

    print("vehicle_current_speed______", vehicle_current_speed)

    # 判断汽车当前是否处于红绿灯影响范围
    if vehicle.is_at_traffic_light():
        # 获取红绿灯
        traffic_light = vehicle.get_traffic_light()
        print("traffic_light.get_state()______", traffic_light.get_state())
        # 如果前方是红灯并且距离路口只有0.5m了，制动汽车
        if traffic_light and traffic_light.get_state() == carla.TrafficLightState.Red:
            print("红灯请停车！")
            # target_velocity = max(vehicle_current_speed - 5.0, 0.0)  # 以5m/s的速度减速，
            vehicle.apply_control(carla.VehicleControl(throttle=-0.0, steer=0, brake=1.0))
        # 如果前方是绿灯
        elif traffic_light and traffic_light.get_state() == carla.TrafficLightState.Green:
            # 如果当前是静止的状态，则让汽车启动
            if vehicle_current_speed == 0.0:
                print("绿灯请行驶！")
                vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0, brake=0.0))
            # 如果当前是行驶状态并且速度大于最大速度限制的一半，则让其保持当前速度行驶
            elif vehicle_current_speed > (vehicle_max_speed / 2.0):
                print("路口请缓行！")
                vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.0))
            # 其他情况，则保持当前速度行驶
            else :
                print("绿灯请缓行！")
                vehicle.apply_control(carla.VehicleControl(throttle=0.5, steer=0.0, brake=0.0))
        # 如果前方是黄灯，则汽车开始制动
        elif traffic_light and traffic_light.get_state() == carla.TrafficLightState.Yellow:
            print("黄灯请停车！")
            vehicle.apply_control(carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0))
    else:
        print("直线行驶中~~~")
        # 如果汽车的速度超过了最大速度，则让他保持当前速度行驶
        speed_tolerance = 0.5  # 设置速度容忍范围，可以根据需要调整
        # 计算油门值，使车辆保持在最大速度附近
        if vehicle_current_speed < vehicle_max_speed - speed_tolerance:
            # print("start throttle")
            throttle = 1.0  # 假设油门力度为1.0
        else:
            # print("stop throttle")
            throttle = 0.0  # 达到最大速度时，停止加油门

        vehicle.apply_control(carla.VehicleControl(throttle=throttle, steer=0.0, brake=0.0))


def main():
    actor_list = []
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(2.0)
    try:
        # 获取世界
        world = client.get_world()

        # 创建汽车蓝图
        # vehicle_blueprint = random.choice(world.get_blueprint_library().filter('vehicle.*'))
        # 选用自定义小车蓝图
        vehicle_blueprint = world.get_blueprint_library().filter('vehicle.simplecar.simplecar')
        # vehicle_spawn_points = random.choice(world.get_map().get_spawn_points())
        vehicle_spawn_points = carla.Transform(
            carla.Location(x=20.235275, y=13.414804, z=0.600000),  # 设置初始位置的x、y和z坐标
            carla.Rotation(pitch=0.000000, yaw=-179.840790, roll=0.000000)  # 设置初始方向的pitch、yaw和roll角度
        )
        print("vehicle_spawn_points: ",vehicle_spawn_points)

        vehicle = world.spawn_actor(vehicle_blueprint, vehicle_spawn_points)
        # 设置最大车速
        vehicle_max_speed = 7.0

        actor_list.append(vehicle)

        # 创建相机蓝图
        camera_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
        camera_spawn_points = carla.Transform(carla.Location(x=-5, z=4), carla.Rotation(pitch=-20))
        camera = world.spawn_actor(camera_blueprint, camera_spawn_points, attach_to=vehicle)
        # camera.listen(lambda image: image.save_to_disk(os.path.join('_out', '%06d.png' % image.frame)))

        actor_list.append(camera)

        while True:
            # 设置相机视角
            world.get_spectator().set_transform(camera.get_transform())

            hld(world, vehicle, vehicle_max_speed)

            # 优化循环性能
            world.tick()

    finally:
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        print("All the actors have already been destroied !")


if __name__ == "__main__":
    main()
