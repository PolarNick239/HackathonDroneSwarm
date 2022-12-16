from station import load_stations
from drone import load_drones
from world import World
from mission import Mission
import random

import cv2


if __name__ == '__main__':
    window_height = 1000

    world = World("data/world.json", window_height)
    control_station, charge_stations = load_stations("data/stations.json")
    drone_master, drones = load_drones("data/drones.json", control_station.x, control_station.y, world)

    world.addDrones(drone_master, drones)
    world.addStations(control_station, charge_stations)

    mission_list = [Mission(key + 1, 10000, random.random() * 22500, random.random() * 22500) for key in range(10)]

    for key, drone in drones.items():
        drone.setMissionList(mission_list)

    while True:
        frame = world.drawDEM()
        dt = world.simulation_step

        drone_master.tryToScheduleTask(drones)
        for key, drone in drones.items():
            drone.update(world, dt)

        world.drawStations(frame)
        world.drawDrones(frame)

        cv2.imshow("Map", frame)
        key = cv2.waitKey(1000//60)  # lock to 60 fps
        if key == 27:  # Escape
            break
    cv2.destroyAllWindows()
