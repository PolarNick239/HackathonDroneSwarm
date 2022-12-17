from station import load_stations
from drone import load_drones
from world import World
import colors
from mission import Mission, MissionPoly, MissionPath, splitMission, load_missions
import random

import cv2


if __name__ == '__main__':
    window_height = 1000

    world = World("data/world.json", window_height)
    control_station, charge_stations = load_stations("data/stations.json")
    drones = load_drones("data/drones.json", control_station.x, control_station.y, world)

    mission_step = 500
    mission_list = load_missions("data/missions.json", mission_step, control_station, world)
    mission_list_split = []
    for mission in mission_list:
        if isinstance(mission, MissionPoly):
            mission_list_split += splitMission(mission, 1000, 8)
        else:
            mission_list_split.append(mission)
    mission_list = mission_list_split
    for i, mission in enumerate(mission_list):
        mission.key = i+1

    world.addDrones(drones)
    world.addStations(control_station, charge_stations)

    window_name = "Drones Swarm Simulator"
    cv2.namedWindow(window_name, (cv2.WINDOW_AUTOSIZE if window_height < 1200 else cv2.WINDOW_NORMAL) | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_NORMAL)

    is_paused = False
    steps_per_frame = 1
    slowdown = 8

    print("__________________________________")
    print("Welcome to {}!".format(window_name))
    print("Controls:")
    print(" SPACE - pause/unpause")
    print(" +/-   - speedup/slowdown simulation")
    print("__________________________________")

    poly_missions = []
    path_missions = []
    for mission in mission_list:
        if isinstance(mission, MissionPath):
            path_missions.append(mission)
        if isinstance(mission, MissionPoly):
            poly_missions.append(mission)

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
                master_drone.tryToScheduleTasks(available_drones, charge_stations, world)
                for key in sorted(drones.keys()):
                    drone = drones[key]
                    drone.update(world, dt / slowdown)

        world.drawStations(frame)
        world.drawPolygonMissions(frame, poly_missions) #TODO move to world?
        world.drawPathMissions(frame, path_missions) #TODO move to world?
        world.drawDrones(frame)

        cv2.putText(frame, "PAUSE (press SPACE BAR)" if is_paused else "x{}".format("1/{}".format(slowdown) if slowdown > 1 else steps_per_frame), (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, colors.BLACK, 1, 2)

        cv2.imshow(window_name, frame)
        key = cv2.waitKey(1000//60)  # lock to 60 fps
        if key == 27:  # Escape
            break
        elif key == 32:  # Space bar
            if not is_paused:
                is_paused = True
            else:
                is_paused = False
        elif key == 43:  # +
            if slowdown > 1:
                slowdown //= 2
            else:
                steps_per_frame *= 2
        elif key == 45:  # -
            if steps_per_frame != 1:
                steps_per_frame = max(1, steps_per_frame // 2)
            else:
                slowdown *= 2
        elif key != -1:
            # print("GUI: Unhandled key: {}".format(key))
            pass
    cv2.destroyAllWindows()
