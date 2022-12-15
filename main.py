from station import load_stations
from drone import load_drones
from world import World

import cv2


if __name__ == '__main__':
    window_height = 1000

    world = World("data/world.json", window_height)
    control_station, charge_stations = load_stations("data/stations.json")
    drone_master, drones = load_drones("data/drones.json", control_station.x, control_station.y, world)

    world.addDrones(drone_master, drones)
    world.addStations(control_station, charge_stations)

    while True:
        frame = world.drawDEM()
        dt = world.simulation_step

        drone_master.tryToScheduleTask(drones)
        for key, drone in drones.items():
            drone.fly(dt)

        world.drawStations(frame)
        world.drawDrones(frame)

        cv2.imshow("Map", frame)
        key = cv2.waitKey(1000//60)  # lock to 60 fps
        if key == 27:  # Escape
            break
    cv2.destroyAllWindows()
