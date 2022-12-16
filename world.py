import json
import numpy as np
from PIL import Image
import colors

import cv2


class World:

    def __init__(self, json_path, window_height) -> None:
        with open(json_path, "r") as file:
            world_data = json.load(file)
        assert "world" in world_data
        world_data = world_data["world"]
        self.dem_resolution = world_data["dem_resolution"]  # meters/pixel
        dem_path = world_data["dem_path"]
        self.dem_image = Image.open(dem_path)
        self.maximum_allowed_height = world_data["maximum_allowed_height"]
        self.simulation_step = world_data["simulation_step"]  # in seconds
        self.wireless_range = world_data["wireless_range"]  # in meters
        self.drones_speed = world_data["drone_speed"]  # m/s
        self.drone_lifetime = world_data["drone_life"]  # in seconds
        self.charge_power = world_data["charge_power"]  # in seconds of flight per second of charge

        self.dem_image_scale_ratio = window_height // self.dem_image.height
        self.window_height = self.dem_image.height * self.dem_image_scale_ratio
        self.window_width = self.dem_image.width * self.dem_image_scale_ratio
        print("DEM loaded: {}x{} pixels, {}x{} m"
              .format(self.dem_image.width, self.dem_image.height,
                      int(self.dem_image.width * self.dem_resolution), int(self.dem_image.height * self.dem_resolution)))

        self.drone_master = None
        self.drones = None
        self.control_station = None
        self.charge_stations = None

    def addDrones(self, drone_master, drones):
        self.drone_master = drone_master
        self.drones = drones

    def addStations(self, control_station, charge_stations):
        self.control_station = control_station
        self.charge_stations = charge_stations

    def toWindowPixel(self, x, y):
        ratio = self.dem_image_scale_ratio / self.dem_resolution
        return int(x * ratio), int(y * ratio)

    def drawDEM(self):
        # TODO draw red zones (w.r.t. self.maximum_allowed_height)
        image = self.dem_image.convert('RGB')
        image = np.array(image)
        image = cv2.resize(image, (self.window_width, self.window_height))
        return image

    def drawDrones(self, frame):
        self.drawWirelessNetwork(frame)

        drone_radius = 5
        master_color = colors.RED
        drone_color = colors.BLUE

        for key, drone in self.drones.items():
            x, y = self.toWindowPixel(drone.x, drone.y)
            cv2.circle(frame, (x, y), drone_radius, master_color if drone.is_master else drone_color)

            text = "{} {}".format(key, drone.state)
            font = cv2.FONT_HERSHEY_SIMPLEX
            bottomLeftCornerOfText = (x + 10, y)
            fontColor = colors.BLACK
            fontScale, thickness, lineType = 1, 1, 2
            cv2.putText(frame, text, bottomLeftCornerOfText, font, fontScale, fontColor, thickness, lineType)
            if drone.targetX is not None:
                cv2.line(frame, self.toWindowPixel(drone.x, drone.y), self.toWindowPixel(drone.targetX, drone.targetY), colors.BLUE)

    def drawStations(self, frame):
        station_radius = 10
        station_thickness = -1
        control_station_color = colors.BLUE
        charge_station_color = colors.GREEN

        x, y = self.toWindowPixel(self.control_station.x, self.control_station.y)
        cv2.circle(frame, (x, y), station_radius, control_station_color, station_thickness)

        for key, station in self.charge_stations.items():
            x, y = self.toWindowPixel(station.x, station.y)
            cv2.circle(frame, (x, y), station_radius, charge_station_color, station_thickness)

    def drawWirelessNetwork(self, frame):
        from scipy.sparse import csr_matrix
        from scipy.sparse.csgraph import minimum_spanning_tree

        # See https://docs.scipy.org/doc/scipy/reference/generated/scipy.sparse.csgraph.minimum_spanning_tree.html
        matrix = []
        keys = sorted(self.drones.keys())
        for i0, key0 in enumerate(keys):
            neighbors0 = []
            drone0 = self.drones[key0]
            for i1, key1 in enumerate(keys):
                drone1 = self.drones[key1]
                if i1 <= i0:
                    distance = 0
                else:
                    distance = drone0.distanceTo(drone1.x, drone1.y)
                neighbors0.append(distance)
            matrix.append(neighbors0)
        matrix = csr_matrix(matrix)

        spanning_tree_matrix = minimum_spanning_tree(matrix)
        spanning_tree_matrix = spanning_tree_matrix.toarray().astype(int)

        for i0, key0 in enumerate(keys):
            drone0 = self.drones[key0]
            for i1, key1 in enumerate(keys):
                drone1 = self.drones[key1]
                distance = spanning_tree_matrix[i0][i1]
                if distance == 0:
                    continue
                cv2.line(frame, self.toWindowPixel(drone0.x, drone0.y), self.toWindowPixel(drone1.x, drone1.y),
                         colors.GREEN if distance <= self.wireless_range else colors.RED)
