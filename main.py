from station import load_stations
from drone import load_drones
from world import World
import colors

import cv2


if __name__ == '__main__':
    window_height = 1000

    world = World("data/world.json", window_height)
    control_station, charge_stations = load_stations("data/stations.json")
    drone_master, drones = load_drones("data/drones.json", control_station.x, control_station.y, world)

    world.addDrones(drone_master, drones)
    world.addStations(control_station, charge_stations)

    window_name = "Drones Swarm Simulator"
    cv2.namedWindow(window_name, (cv2.WINDOW_AUTOSIZE if window_height < 1200 else cv2.WINDOW_NORMAL) | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_NORMAL)

    is_paused = False
    steps_per_frame = 1

    while True:
        frame = world.drawDEM()
        dt = world.simulation_step

        if not is_paused:
            for step in range(steps_per_frame):
                drone_master.tryToScheduleTask(drones)
                for key in sorted(drones.keys()):
                    drone = drones[key]
                    drone.update(world, dt)

        world.drawStations(frame)
        world.drawDrones(frame)

        cv2.putText(frame, "PAUSE" if is_paused else "x{}".format(steps_per_frame), (0, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, colors.BLACK, 1, 2)

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
