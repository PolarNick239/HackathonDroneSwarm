import json
import numpy as np
from PIL import Image

import cv2

BLACK = (0, 0, 0)
RED = (0, 0, 255)
GREEN = (0, 255, 0)
BLUE = (255, 0, 0)


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
        self.drones_speed = world_data["drone_speed"]  # m/s
        self.drone_lifetime = world_data["drone_life"]  # in seconds

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
        image = self.dem_image.convert('RGB')
        image = np.array(image)
        image = cv2.resize(image, (self.window_width, self.window_height))
        return image

    def drawDrones(self, frame):
        drone_radius = 5
        master_color = RED
        drone_color = BLUE

        x, y = self.toWindowPixel(self.drone_master.x, self.drone_master.y)
        cv2.circle(frame, (x, y), drone_radius, master_color)

        for key, drone in self.drones.items():
            x, y = self.toWindowPixel(drone.x, drone.y)
            cv2.circle(frame, (x, y), drone_radius, drone_color)

            text = "{} {}".format(key, drone.state)
            font = cv2.FONT_HERSHEY_SIMPLEX
            bottomLeftCornerOfText = (x + 10, y)
            fontColor = BLACK
            fontScale, thickness, lineType = 1, 1, 2
            cv2.putText(frame, text, bottomLeftCornerOfText, font, fontScale, fontColor, thickness, lineType)

    def drawStations(self, frame):
        station_radius = 10
        station_thickness = -1
        control_station_color = BLUE
        charge_station_color = GREEN

        x, y = self.toWindowPixel(self.control_station.x, self.control_station.y)
        cv2.circle(frame, (x, y), station_radius, control_station_color, station_thickness)

        for key, station in self.charge_stations.items():
            x, y = self.toWindowPixel(station.x, station.y)
            cv2.circle(frame, (x, y), station_radius, charge_station_color, station_thickness)