import glob
import os
import sys
import time
import random
import time
import numpy as np
import cv2
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla

def main():
    # 连接到CARLA服务器
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)

    # 获取世界
    world = client.get_world()

    # 获取propsFactory
    props_factory = world.get_blueprint_library().filter('*factory*')

    print(props_factory)

    try:
        pass

    finally:
        pass


if __name__ == '__main__':
    main()

    '''
    
[
ActorBlueprint(id=static.prop.atm,tags=[static, prop, atm]), 
ActorBlueprint(id=static.prop.plantpot04,tags=[static, prop, plantpot04]),
ActorBlueprint(id=static.prop.bench02,tags=[bench02, static, prop]), 
ActorBlueprint(id=static.prop.kiosk_01,tags=[static, prop, kiosk_01]), 
ActorBlueprint(id=static.prop.creasedbox03,tags=[static, prop, creasedbox03]), 
ActorBlueprint(id=static.prop.trashcan05,tags=[static, prop, trashcan05]), 
ActorBlueprint(id=static.prop.bike helmet,tags=[static, prop, bike helmet]), 
ActorBlueprint(id=static.prop.oil_station_tips,tags=[oil_station_tips, static, prop]), 
ActorBlueprint(id=static.prop.plantpot06,tags=[plantpot06, static, prop]), 
ActorBlueprint(id=static.prop.briefcase,tags=[briefcase, static, prop]), 
ActorBlueprint(id=static.prop.table,tags=[static, prop, table]), 
ActorBlueprint(id=static.prop.barrel,tags=[static, prop, barrel]), 
ActorBlueprint(id=static.prop.plantpot02,tags=[static, prop, plantpot02]), 
ActorBlueprint(id=static.prop.streetsign,tags=[streetsign, static, prop]), 
ActorBlueprint(id=static.prop.glasscontainer,tags=[static, prop, glasscontainer]), 
ActorBlueprint(id=static.prop.chainbarrier,tags=[static, prop, chainbarrier]), 
ActorBlueprint(id=static.prop.plastictable,tags=[static, prop, plastictable]),
ActorBlueprint(id=static.prop.trashcan04,tags=[static, prop, trashcan04]), 
ActorBlueprint(id=static.prop.fountain,tags=[static, prop, fountain]), 
ActorBlueprint(id=static.prop.creasedbox02,tags=[static, prop, creasedbox02]), 
ActorBlueprint(id=static.prop.pergola,tags=[static, prop, pergola]), 
ActorBlueprint(id=static.prop.slide,tags=[static, prop, slide]), ActorBlueprint(id=static.prop.ironplank,tags=[static, prop, ironplank]), ActorBlueprint(id=static.prop.warningconstruction,tags=[static, prop, warningconstruction]), ActorBlueprint(id=static.prop.colacan,tags=[static, prop, colacan]), ActorBlueprint(id=static.prop.advertisement,tags=[static, prop, advertisement]), ActorBlueprint(id=static.prop.platformgarbage01,tags=[static, prop, platformgarbage01]), ActorBlueprint(id=static.prop.vendingmachine,tags=[vendingmachine, static, prop]), ActorBlueprint(id=static.prop.doghouse,tags=[static, prop, doghouse]), ActorBlueprint(id=static.prop.barbeque,tags=[static, barbeque, prop]), ActorBlueprint(id=static.prop.creasedbox01,tags=[static, prop, creasedbox01]), ActorBlueprint(id=static.prop.chainbarrierend,tags=[static, prop, chainbarrierend]), ActorBlueprint(id=static.prop.trashcan03,tags=[static, prop, trashcan03]), ActorBlueprint(id=static.prop.mailbox,tags=[mailbox, static, prop]), ActorBlueprint(id=static.prop.dirtdebris02,tags=[static, prop, dirtdebris02]), ActorBlueprint(id=static.prop.garbage05,tags=[garbage05, static, prop]), ActorBlueprint(id=static.prop.garbage04,tags=[static, garbage04, prop]), ActorBlueprint(id=static.prop.busstop,tags=[static, prop, busstop]), ActorBlueprint(id=static.prop.plasticbag,tags=[plasticbag, static, prop]), ActorBlueprint(id=static.prop.clothcontainer,tags=[static, prop, clothcontainer]), ActorBlueprint(id=static.prop.swingcouch,tags=[static, prop, swingcouch]), ActorBlueprint(id=static.prop.bench01,tags=[static, prop, bench01]), ActorBlueprint(id=static.prop.plantpot07,tags=[static, plantpot07, prop]), ActorBlueprint(id=static.prop.bench03,tags=[static, bench03, prop]), ActorBlueprint(id=static.prop.plantpot05,tags=[static, prop, plantpot05]), ActorBlueprint(id=static.prop.bin,tags=[static, prop, bin]), ActorBlueprint(id=static.prop.box01,tags=[static, prop, box01]), ActorBlueprint(id=static.prop.garbage03,tags=[static, prop, garbage03]), ActorBlueprint(id=static.prop.box02,tags=[static, prop, box02]), ActorBlueprint(id=static.prop.box03,tags=[static, prop, box03]), ActorBlueprint(id=static.prop.garbage01,tags=[static, prop, garbage01]), ActorBlueprint(id=static.prop.brokentile01,tags=[static, prop, brokentile01]), ActorBlueprint(id=static.prop.brokentile02,tags=[static, prop, brokentile02]), ActorBlueprint(id=static.prop.brokentile03,tags=[static, prop, brokentile03]), ActorBlueprint(id=static.prop.trafficcone02,tags=[static, prop, trafficcone02]), ActorBlueprint(id=static.prop.brokentile04,tags=[static, brokentile04, prop]), ActorBlueprint(id=static.prop.clothesline,tags=[static, prop, clothesline]), ActorBlueprint(id=static.prop.shoppingbag,tags=[static, prop, shoppingbag]), ActorBlueprint(id=static.prop.constructioncone,tags=[static, prop, constructioncone]), ActorBlueprint(id=static.prop.container,tags=[static, container, prop]), ActorBlueprint(id=static.prop.dirtdebris01,tags=[static, prop, dirtdebris01]), ActorBlueprint(id=static.prop.dirtdebris03,tags=[static, prop, dirtdebris03]), ActorBlueprint(id=static.prop.garbage02,tags=[static, prop, garbage02]), ActorBlueprint(id=static.prop.plasticchair,tags=[static, prop, plasticchair]), ActorBlueprint(id=static.prop.garbage06,tags=[static, prop, garbage06]), ActorBlueprint(id=static.prop.calibrator,tags=[static, prop, calibrator]), ActorBlueprint(id=static.prop.gardenlamp,tags=[static, prop, gardenlamp]), ActorBlueprint(id=static.prop.gnome,tags=[static, prop, gnome]), ActorBlueprint(id=static.prop.guitarcase,tags=[guitarcase, static, prop]), ActorBlueprint(id=static.prop.maptable,tags=[maptable, static, prop]), ActorBlueprint(id=static.prop.mobile,tags=[static, prop, mobile]), ActorBlueprint(id=static.prop.plantpot08,tags=[static, prop, plantpot08]), ActorBlueprint(id=static.prop.motorhelmet,tags=[static, prop, motorhelmet]), ActorBlueprint(id=static.prop.plantpot01,tags=[static, prop, plantpot01]), ActorBlueprint(id=static.prop.plantpot03,tags=[static, prop, plantpot03]), ActorBlueprint(id=static.prop.purse,tags=[static, purse, prop]), ActorBlueprint(id=static.prop.shoppingcart,tags=[static, prop, shoppingcart]), ActorBlueprint(id=static.prop.shoppingtrolley,tags=[static, prop, shoppingtrolley]), ActorBlueprint(id=static.prop.streetsign01,tags=[static, streetsign01, prop]), ActorBlueprint(id=static.prop.haybalelb,tags=[static, prop, haybalelb]), ActorBlueprint(id=static.prop.streetsign04,tags=[static, prop, streetsign04]), ActorBlueprint(id=static.prop.streetbarrier,tags=[static, prop, streetbarrier]), ActorBlueprint(id=static.prop.streetfountain,tags=[static, prop, streetfountain]), ActorBlueprint(id=static.prop.swing,tags=[static, prop, swing]), ActorBlueprint(id=static.prop.trafficcone01,tags=[static, prop, trafficcone01]), ActorBlueprint(id=static.prop.trafficwarning,tags=[static, prop, trafficwarning]), ActorBlueprint(id=static.prop.trampoline,tags=[static, prop, trampoline]), ActorBlueprint(id=static.prop.trashbag,tags=[static, prop, trashbag]), ActorBlueprint(id=static.prop.trashcan01,tags=[static, prop, trashcan01]), ActorBlueprint(id=static.prop.trashcan02,tags=[static, prop, trashcan02]), ActorBlueprint(id=static.prop.travelcase,tags=[static, prop, travelcase]), ActorBlueprint(id=static.prop.wateringcan,tags=[static, prop, wateringcan]), ActorBlueprint(id=static.prop.foodcart,tags=[foodcart, static, prop]), ActorBlueprint(id=static.prop.haybale,tags=[haybale, static, prop]), ActorBlueprint(id=static.prop.busstoplb,tags=[static, prop, busstoplb]), ActorBlueprint(id=static.prop.mesh,tags=[static, prop, mesh]), ActorBlueprint(id=static.prop.warningaccident,tags=[static, prop, warningaccident]), ActorBlueprint(id=static.prop.oil_station_house,tags=[oil_station_house, static, prop]), ActorBlueprint(id=static.prop.oil_station_oil,tags=[oil_station_oil, static, prop])]

    '''