#!/usr/bin/env python
'''
1. 配置Carla仿真平台https://github.com/carla-simulator/carla
2. 在Carla仿真平台上完成路网设置、红绿灯、车辆和交通参与者的设置
3. 在Carla仿真平台上完成车辆跟车、等红绿灯算法的仿真
'''
from __future__ import print_function

# ==============================================================================
# -- find carla module ---------------------------------------------------------
# ==============================================================================


import glob
import os
import random
import sys
import carla

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass






def create_scenario():
    actor_list = []

    try:
        client = carla.Client(host='127.0.0.1', port=2000)
        client.set_timeout(2.0)
        world = client.get_world() 
        # 创建红绿灯
        traffic_light_bp = random.choice(world.get_blueprint_library().filter('traffic.traffic_light'))
        traffic_light_location = carla.Location(x=100, y=200, z=0)  # 设置红绿灯的位置
        traffic_light = world.spawn_actor(traffic_light_bp, traffic_light_location)
        traffic_light.set_signal_state(carla.TrafficLightState.Green)  # 设置绿灯
        actor_list.append(traffic_light)

        # 创建车辆
        vehicle_bp = random.choice(world.get_blueprint_library().filter('vehicle.*'))
        vehicle_location = carla.Location(x=50, y=200, z=0)  # 设置车辆的初始位置
        vehicle = world.spawn_actor(vehicle_bp, vehicle_location)
        actor_list.append(vehicle)

        # 创建交通参与者（行人）
        walker_bp = random.choice(world.get_blueprint_library().filter('walker.pedestrian.*'))
        walker_location = carla.Location(x=150, y=200, z=0)  # 设置行人的初始位置
        walker = world.spawn_actor(walker_bp, walker_location)
        actor_list.append(walker)

        # 设置行人的目标位置
        target_location = carla.Location(x=250, y=200, z=0)
        walker_ai_controller = carla.WalkerAIController(walker, 1.0)
        walker_ai_controller.set_target_location(target_location)

    finally:
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        print("Done. Cleaning up...")


if __name__ == '__main__':
    create_scenario()
