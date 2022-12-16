import json
import numpy as np
from PIL import Image
import colors
import networkx as nx
from utils import dist, simplifyPath

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

        self.prepairPathPlanning()

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



    def drawPathMissions(self, frame, missions):

        waypoint_radius = 2
        waypoint_thickness = 2

        for mission in missions:
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

    def toVertexId(self, i, j):
        return j * self.dem_image.width + i

    def fromVertexId(self, vertexId):
        i = vertexId % self.dem_image.width
        j = vertexId // self.dem_image.width
        return i, j

    def prepairPathPlanning(self):
        nvertices = self.dem_image.width * self.dem_image.height
        print("building {} vertices graph w.r.t. DEM...".format(nvertices))
        self.g = nx.Graph()
        ignore_dem = False  # for debug/development speedup
        for j in range(self.dem_image.height - 1):
            for i in range(self.dem_image.width - 1):
                if ignore_dem:
                    v0 = self.toVertexId(i, j)
                    v1 = self.toVertexId(i+1, j)
                    self.g.add_edge(v0, v1, weight=1)
                    v1 = self.toVertexId(i, j+1)
                    self.g.add_edge(v0, v1, weight=1)
                    v1 = self.toVertexId(i+1, j+1)
                    self.g.add_edge(v0, v1, weight=1.41)
                else:
                    if self.dem_prohibited_mask[j, i]:
                        continue
                    # for dj in range(2):
                    #     for di in range(-1, 2):
                    for dj in range(3):
                        for di in range(-2, 3):
                            if di == 0 and dj == 0:
                                continue
                            if i + di < 0 or i + di >= self.dem_image.width or j + dj >= self.dem_image.height:
                                continue
                            if self.dem_prohibited_mask[j + dj, i + di]:
                                continue
                            if abs(di) == 2 and dj == 0 and self.dem_prohibited_mask[j + 0, i + di // 2]:
                                continue
                            if abs(di) == 2 and dj == 1 and self.dem_prohibited_mask[j + 0, i + di // 2] and \
                                    self.dem_prohibited_mask[j + 1, i + di // 2]:
                                continue
                            if abs(di) == 1 and dj == 2 and self.dem_prohibited_mask[j + dj // 2, i + 0] and \
                                    self.dem_prohibited_mask[j + dj // 2, i + di]:
                                continue
                            if abs(di) == 0 and dj == 2 and self.dem_prohibited_mask[j + 1, i + 0]:
                                continue
                            distance = dist(di * self.dem_resolution, dj * self.dem_resolution)
                            v0 = self.toVertexId(i, j)
                            v1 = self.toVertexId(i + di, j + dj)
                            v0, v1 = min(v0, v1), max(v0, v1)
                            self.g.add_edge(v0, v1, weight=distance)
        self.cachedPaths = {}
        print("graph prepaired!")

    def estimatePath(self, x0, y0, x1, y1):
        start = (x0, y0)
        finish = (x1, y1)

        # to DEM image pixels coordinates:
        x0, y0 = x0 // self.dem_resolution, y0 // self.dem_resolution
        x1, y1 = x1 // self.dem_resolution, y1 // self.dem_resolution
        assert x0 >= 0 and x0 < self.dem_image.width and x1 >= 0 and x1 < self.dem_image.width
        assert y0 >= 0 and y0 < self.dem_image.height and y1 >= 0 and y1 < self.dem_image.height

        startId = self.toVertexId(x0, y0)
        finishId = self.toVertexId(x1, y1)

        key = (startId, finishId)
        if key in self.cachedPaths:
            xys = self.cachedPaths[key]
            assert xys[0] == start
            assert xys[-1] == finish
            return xys

        vertices = nx.shortest_path(self.g, source=startId, target=finishId, weight='weight')
        assert vertices[0] == startId
        assert vertices[-1] == finishId

        xys = []
        for curId in vertices:
            i, j = self.fromVertexId(curId)
            if curId == startId:
                x, y = start
            elif curId == finishId:
                x, y = finish
            else:
                x, y = (i + 0.5) * self.dem_resolution, (j + 0.5) * self.dem_resolution
            xys.append((x, y))
        assert xys[0] == start
        assert xys[-1] == finish
        max_error = (self.dem_resolution / 4.0)
        xys = simplifyPath(xys, max_error)
        assert xys[0] == start
        assert xys[-1] == finish
        self.cachedPaths[key] = xys
        return xys

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
