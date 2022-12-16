from station import load_stations
from drone import load_drones
from world import World
import colors
from mission import Mission, MissionPoly, MissionPath
import random

import cv2

def polySquare(dx, dy):
    return [(10000+dx, 5000+dy), (15000+dx, 5000+dy), (15000+dx, 10000+dy), (10000+dx, 10000+dy)]

if __name__ == '__main__':
    window_height = 1000

    world = World("data/world.json", window_height)
    control_station, charge_stations = load_stations("data/stations.json")
    drones = load_drones("data/drones.json", control_station.x, control_station.y, world)

    world.addDrones(drones)
    world.addStations(control_station, charge_stations)

    window_name = "Drones Swarm Simulator"
    cv2.namedWindow(window_name, (cv2.WINDOW_AUTOSIZE if window_height < 1200 else cv2.WINDOW_NORMAL) | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_NORMAL)

    is_paused = True
    steps_per_frame = 1

    print("__________________________________")
    print("Welcome to {}!".format(window_name))
    print("Controls:")
    print(" SPACE - pause/unpause")
    print(" +/-   - speedup/slowdown simulation")
    print("__________________________________")

    poly_missions = []
    poly_missions.append(MissionPoly(0, "scan", polySquare(0, 0), 1000))
    poly_missions.append(MissionPoly(1, "scan", polySquare(5000, 5000), 1000))
    poly_missions.append(MissionPoly(2, "scan", polySquare(0, 5000), 1000))
    poly_missions.append(MissionPoly(3, "scan", polySquare(5000, 0), 1000))
    poly_missions.append(MissionPoly(4, "scan", [(10000, 15000), (15000, 20000), (5000, 20000)], 1000))

    path_missions = []
    path = world.estimatePath(16000, 17000, 30*world.dem_resolution, 25*world.dem_resolution) + world.estimatePath(30 * world.dem_resolution, 25*world.dem_resolution, 16000, 17000)
    path_missions.append(MissionPath(2, "path", path))

    mission_list = []
    mission_list += poly_missions
    mission_list += path_missions

    # mission_list = [Mission(key + 1, 10000, random.random() * 22500, random.random() * 22500) for key in range(10)]

    for key, drone in drones.items():
        drone.setMissionList(mission_list)

    while True:
        frame = world.drawDEM()
        dt = world.simulation_step

        if not is_paused:
            for step in range(steps_per_frame):
                master_drone = world.getMasterDrone()
                available_drones = world.getWirelessReachableDrones(master_drone)
                master_drone.tryToScheduleTask(available_drones)
                for key in sorted(drones.keys()):
                    drone = drones[key]
                    drone.update(world, dt)

        world.drawStations(frame)
        world.drawDrones(frame)
        world.drawPolygonMissions(frame, poly_missions) #TODO move to world?
        world.drawPathMissions(frame, path_missions) #TODO move to world?

        cv2.putText(frame, "PAUSE (press SPACE BAR)" if is_paused else "x{}".format(steps_per_frame), (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, colors.BLACK, 1, 2)

        cv2.imshow(window_name, frame)
        key = cv2.waitKey(1000//60)  # lock to 60 fps
        if key == 27:  # Escape
            break
        elif key == 32:  # Space bar
            if not is_paused:
                is_paused = True
                print("GUI: Paused!")
            else:
                is_paused = False
                print("GUI: Un-paused!")
        elif key == 43:  # +
            steps_per_frame *= 2
            print("GUI: speedup to x{} steps per frame".format(steps_per_frame))
        elif key == 45:  # -
            if steps_per_frame != 1:
                steps_per_frame = max(1, steps_per_frame // 2)
                print("GUI: slow down to x{} steps per frame".format(steps_per_frame))
        elif key != -1:
            print("GUI: Unhandled key: {}".format(key))
    cv2.destroyAllWindows()
