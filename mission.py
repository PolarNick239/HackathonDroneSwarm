
import numpy as np

from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

class Mission:

    def __init__(self, key, total_time, x, y):
        self.key = key
        self.total_time = total_time
        self.time_to_finish_left = total_time
        self.waypoints = [(x, y)]

    def update(self, dt):
        self.time_to_finish_left = max(0, self.time_to_finish_left - dt)

    def finished(self):
        return self.time_to_finish_left == 0



def rasterizePolygon(polygon, step):
    assert polygon is not None and len(polygon) > 0

    minx, miny = polygon[0]
    maxx, maxy = minx, miny
    for x, y in polygon:
        minx = min(x, minx)
        miny = min(y, miny)
        maxx = max(x, maxx)
        maxy = max(y, maxy)

    waypoints = []

    poly = Polygon(polygon)

    for irow, y in enumerate(np.arange(miny, maxy, step)):
        row = []
        for x in np.arange(minx, maxx, step):
            if poly.contains(Point(x, y)):
                row.append((x, y))

        if irow % 2 == 0:
            row.reverse()

        waypoints += row

    return waypoints

class MissionPoly:

    def __init__(self, key, type, polygon, step):
        self.key = key
        self.type = type
        self.polygon = polygon
        self.waypoints = rasterizePolygon(polygon, step)
        self.waypoint_visited = [False for _ in self.waypoints]
        self.n_waypoints_visited = 0

    def update(self, dt):
        self.waypoint_visited[self.n_waypoints_visited] = True
        self.n_waypoints_visited += 1

    def finished(self):
        return self.n_waypoints_visited >= len(self.waypoints)

    def hasNextWaypoint(self):
        return not self.finished()

    def nextWaypoint(self):
        return self.waypoints[self.n_waypoints_visited]



class MissionPath:

    def __init__(self, key, type, path):
        self.key = key
        self.type = type
        self.waypoints = path
        self.waypoint_visited = [False for _ in self.waypoints]
        self.n_waypoints_visited = 0

    def update(self, dt):
        self.waypoint_visited[self.n_waypoints_visited] = True
        self.n_waypoints_visited += 1

    def finished(self):
        return self.n_waypoints_visited >= len(self.waypoints)

    def hasNextWaypoint(self):
        return not self.finished()

    def nextWaypoint(self):
        return self.waypoints[self.n_waypoints_visited]
