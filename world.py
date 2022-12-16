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
        self.maximum_allowed_height = world_data["maximum_allowed_height"]
        self.dem_image = Image.open(dem_path)
        self.dem_prohibited_mask = self.estimateProhibitedDEMMask()
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

        self.drones = None
        self.control_station = None
        self.charge_stations = None

    def addDrones(self, drones):
        self.drones = drones

    def getMasterDrone(self):
        master_drone = None
        for key, drone in self.drones.items():
            if drone.is_master:
                assert master_drone is None
                master_drone = drone
        return master_drone

    def addStations(self, control_station, charge_stations):
        self.control_station = control_station
        self.charge_stations = charge_stations

    def toWindowPixel(self, x, y):
        ratio = self.dem_image_scale_ratio / self.dem_resolution
        return int(x * ratio), int(y * ratio)

    def estimateProhibitedDEMMask(self):
        image = self.dem_image.convert('RGB')
        image = np.array(image)

        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        is_prohibited = gray_image > self.maximum_allowed_height
        return is_prohibited

    def drawDEM(self):
        image = self.dem_image.convert('RGB')
        image = np.array(image)

        image[self.dem_prohibited_mask] = colors.RED

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

            text = "{} {}".format(key + ("M" if drone.is_master else ""), drone.state)
            font = cv2.FONT_HERSHEY_SIMPLEX
            bottomLeftCornerOfText = (x + 10, y)
            fontColor = colors.BLACK
            fontScale, thickness, lineType = 1, 1, 2
            cv2.putText(frame, text, bottomLeftCornerOfText, font, fontScale, fontColor, thickness, lineType)
            if drone.targetX is not None:
                cv2.line(frame, self.toWindowPixel(drone.x, drone.y), self.toWindowPixel(drone.targetX, drone.targetY), colors.BLUE)

    def drawPolygonMissions(self, frame, missions):

        waypoint_radius = 2
        waypoint_thickness = 2

        for mission in missions:
            polygon = mission.polygon
            n = len(polygon)
            for i in range(n):
                j = (i+1)%n
                x0 = polygon[i][0]
                y0 = polygon[i][1]
                x1 = polygon[j][0]
                y1 = polygon[j][1]
                cv2.line(frame, self.toWindowPixel(x0, y0), self.toWindowPixel(x1, y1), colors.CYAN)

            for i, waypoint in enumerate(mission.waypoints):

                x, y = self.toWindowPixel(waypoint[0], waypoint[1])
                color = colors.CYAN if mission.waypoint_visited[i] else colors.YELLOW

                if i > 0:
                    xprev, yprev = self.toWindowPixel(mission.waypoints[i-1][0], mission.waypoints[i-1][1])
                    cv2.line(frame, (xprev, yprev), (x, y), color)

                cv2.circle(frame, (x, y), waypoint_radius, color, waypoint_thickness)



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

    # def estimatePath(self, x0, y0, x1, y1):


    def generateWirelessNetworkSpanningTree(self):
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
        return spanning_tree_matrix

    def drawWirelessNetwork(self, frame):
        spanning_tree_matrix = self.generateWirelessNetworkSpanningTree()
        keys = sorted(self.drones.keys())
        for i0, key0 in enumerate(keys):
            drone0 = self.drones[key0]
            for i1, key1 in enumerate(keys):
                drone1 = self.drones[key1]
                distance = spanning_tree_matrix[i0][i1]
                if distance == 0:
                    continue
                cv2.line(frame, self.toWindowPixel(drone0.x, drone0.y), self.toWindowPixel(drone1.x, drone1.y),
                         colors.GREEN if distance <= self.wireless_range else colors.RED)

    def getWirelessReachableDrones(self, drone):
        spanning_tree_matrix = self.generateWirelessNetworkSpanningTree()
        keys = sorted(self.drones.keys())
        for iStart, keyStart in enumerate(keys):
            if keyStart != drone.key:
                continue
            assert keyStart == drone.key
            is_reachable = [False] * len(keys)

            is_reachable[iStart] = True
            changes_happend = True
            while changes_happend:
                changes_happend = False
                for i0, key0 in enumerate(keys):
                    if not is_reachable[i0]:
                        continue
                    for i1, key1 in enumerate(keys):
                        if is_reachable[i1]:
                            continue
                        distance = max(spanning_tree_matrix[i0, i1], spanning_tree_matrix[i1, i0])
                        if distance != 0 and distance <= self.wireless_range:
                            is_reachable[i1] = True
                            changes_happend = True

            reachable_drones = {}
            for i, key in enumerate(keys):
                if is_reachable[i]:
                    reachable_drones[key] = self.drones[key]
            assert drone in reachable_drones.values()
            return reachable_drones
