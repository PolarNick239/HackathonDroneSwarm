import json
import random

from mission import Mission, MissionPoly, MissionPath, MissionPatrol
from utils import *


class Drone:

    def __init__(self, drone_data, start_x, start_y, charge_power):
        self.is_master = drone_data["isMaster"]
        self.payload = drone_data["payload"]
        self.x = start_x
        self.y = start_y
        self.key = None

        self.targetX = None  #TODO replace with values from mission
        self.targetY = None  #TODO replace with values from mission
        self.targetMission = None
        self.pathPlannerMission = None

        self.speed = drone_data["speed"]
        self.max_lifetime = drone_data["lifetime"]
        self.lifetime_left = drone_data["lifetime"]
        self.charge_power = charge_power
        self.mission_list = []
        self.flying = False

        if "payloadAgroVolume" in drone_data:
            assert "agro" in self.payload
        if "agro" in self.payload:
            assert "payloadAgroVolume" in drone_data
            self.payloadAgroVolume = drone_data["payloadAgroVolume"]
        else:
            self.payloadAgroVolume = 0.0
        self.payloadAgroVolumeLeft = self.payloadAgroVolume

        self.state = "wait"

    # now one object shared between all drones
    # TODO only master holds mission list, other drones request updates over network
    def setMissionList(self, mission_list):
        self.mission_list = mission_list


    def distanceTo(self, x, y):
        return dist(x - self.x, y - self.y)

    def timeTo(self, x, y):
        distance = self.distanceTo(x, y)
        return distance / self.speed

    def speedLimit(self, vx, vy):
        speed = max(1, dist(vx, vy))
        ratio = self.speed / speed
        return vx * ratio, vy * ratio

    def predictNextPosition(self, dt):
        if self.targetX is None:
            return self.x, self.y
        vx, vy = self.speedLimit(self.targetX - self.x, self.targetY - self.y)
        vx, vy = dt * vx, dt * vy
        distanceToTarget = self.distanceTo(self.targetX, self.targetY)
        if dist(vx, vy) < distanceToTarget:
            return self.x + vx, self.y + vy
        else:
            return self.targetX, self.targetY

    def fly(self, world, dt):
        nextX, nextY = self.predictNextPosition(dt)

        for key, that in world.drones.items():
            assert key == that.key
            thatNextX, thatNextY = that.predictNextPosition(dt)
            if isIntersects(self.x, self.y, nextX, nextY, that.x, that.y, thatNextX, thatNextY):
                if self.key < that.key:
                    print("Drone {}: PAUSE! Collision avoidance with Drone {}!".format(self.key, that.key))
                    nextX, nextY = self.x, self.y

        self.x, self.y = nextX, nextY
        if self.x == self.targetX and self.y == self.targetY:
            self.targetX, self.targetY = None, None

            if self.pathPlannerMission is not None:
                self.pathPlannerMission.update(dt)
                if self.pathPlannerMission.finished():
                    self.pathPlannerMission = None
                elif self.pathPlannerMission.hasNextWaypoint():
                    self.targetX = self.pathPlannerMission.nextWaypoint()[0]
                    self.targetY = self.pathPlannerMission.nextWaypoint()[1]
            if self.pathPlannerMission is not None:
                return

            if self.state == "flyToMission":
                print("Drone {}: executing mission {}...".format(self.key, self.targetMission.key))
                self.state = "onMission"
            elif self.state == "flyToCharge":
                print("Drone {}: on charge...".format(self.key))
                self.state = "onCharge"
                self.flying = False
            else:
                raise Exception("fly(): state={} is incorrect!".format(self.state))

    def updateMission(self, dt):
        assert self.state == "onMission"
        self.targetMission.update(dt)
        if self.targetMission.type == "agro":
            self.payloadAgroVolumeLeft -= self.targetMission.agroVolumePerSecond * dt
            if self.payloadAgroVolumeLeft < 0.0:
                print("Drone {}: fail, we are on mission but have no agro payload left!".format(self.key))
        if self.targetMission.finished():
            # print("Drone {}: mission {} finished".format(self.key, self.targetMission.key))
            if isinstance(self.targetMission, MissionPatrol):
                self.targetMission.reset()
                self.mission_list.append(self.targetMission)
            self.targetMission = None
            self.state = "wait"
        elif self.targetMission.hasNextWaypoint():
            # print("Drone {}: mission {} going to next waypoint".format(self.key, self.targetMission.key))
            self.state = "flyToMission"
            self.targetX = self.targetMission.nextWaypoint()[0]
            self.targetY = self.targetMission.nextWaypoint()[1]

    def updateCharge(self, dt):
        assert self.state == "onCharge"
        self.lifetime_left = min(self.max_lifetime, self.lifetime_left + self.charge_power * dt)
        if self.lifetime_left == self.max_lifetime:
            print("Drone {}: charge finished".format(self.key))
            if self.targetMission is None:
                print("Drone {}: charged! waiting...".format(self.key))
                self.state = "wait"
            else:
                raise Exception("this branch should not be touched")
        if self.payloadAgroVolumeLeft != self.payloadAgroVolume:
            print("Drone {}: Agro payload updated to {}!".format(self.key, self.payloadAgroVolume))
            self.payloadAgroVolumeLeft = self.payloadAgroVolume

    def timeToClosestChargeStationFrom(self, x, y, charge_stations):
        closest_station = None
        smallest_time_to_reach = None
        for key, station in charge_stations.items():
            time_to_reach = distbetween(x, y, station.x, station.y) / self.speed
            if smallest_time_to_reach is None or time_to_reach < smallest_time_to_reach:
                closest_station = station
                smallest_time_to_reach = time_to_reach
        return closest_station, smallest_time_to_reach

    def timeToFurthestChargeStationFrom(self, x, y, charge_stations):
        furthest_station = None
        largest_time_to_reach = None
        for key, station in charge_stations.items():
            time_to_reach = distbetween(x, y, station.x, station.y) / self.speed
            if largest_time_to_reach is None or time_to_reach > largest_time_to_reach:
                furthest_station = station
                largest_time_to_reach = time_to_reach
        return furthest_station, largest_time_to_reach

    def checkIfBatteryIsLow(self, charge_stations, world):
        closest_station, smallest_time_to_reach = self.timeToClosestChargeStationFrom(self.x, self.y, charge_stations)

        if smallest_time_to_reach >= self.lifetime_left:
            print("Drone {}: low battery, flying to station {}".format(self.key, closest_station.key))
            self.state = "flyToCharge"
            if self.targetMission is not None:
                self.mission_list.append(self.targetMission)
                self.targetMission = None

            path = world.estimatePath(self.x, self.y, closest_station.x, closest_station.y)
            self.pathPlannerMission = MissionPath(0, "", path)
            self.targetX = self.pathPlannerMission.nextWaypoint()[0]
            self.targetY = self.pathPlannerMission.nextWaypoint()[1]

    def update(self, world, dt):
        if self.state not in {"flyToCharge", "onCharge"}:
            self.checkIfBatteryIsLow(world.charge_stations, world)

        # print('Drone {}: update: drone state: {}, target mission: {}'.format(self.key, self.state, self.targetMission))

        if self.state in {"flyToMission", "flyToCharge"}:
            self.fly(world, dt)
        elif self.state == "onMission":
            self.updateMission(dt)
        elif self.state == "onCharge":
            self.updateCharge(dt)
        elif self.state == "wait":
            # master drone patrols between charging stations to connect to lost drones
            if self.is_master:
                reachable_drones = world.getWirelessReachableDrones(self)
                if len(reachable_drones) < len(world.drones):
                    furthest_station, largest_time_to_reach = self.timeToFurthestChargeStationFrom(self.x, self.y, world.charge_stations)
                    if self.lifetime_left >= largest_time_to_reach:
                        path = world.estimatePath(self.x, self.y, furthest_station.x, furthest_station.y)
                        self.pathPlannerMission = MissionPath(0, "", path)
                        self.targetX = self.pathPlannerMission.nextWaypoint()[0]
                        self.targetY = self.pathPlannerMission.nextWaypoint()[1]
                        self.state = "flyToCharge"
        else:
            raise Exception("state={} is incorrect!".format(self.state))

        if self.flying:
            self.lifetime_left -= dt

    def needTask(self):
        return self.targetMission is None and self.state in {"wait"}

    def addTask(self, mission, world):
        assert self.needTask()

        print("Drone {}: new mission {}".format(self.key, mission.key))
        self.flying = True
        self.targetMission = mission
        if self.state in {"wait"}:
            self.state = "flyToMission"

            path = world.estimatePath(self.x, self.y, mission.nextWaypoint()[0], mission.nextWaypoint()[1])
            self.pathPlannerMission = MissionPath(0, "", path)
            self.targetX = self.pathPlannerMission.nextWaypoint()[0]
            self.targetY = self.pathPlannerMission.nextWaypoint()[1]

    def tryToScheduleTasks(self, available_drones, charge_stations, world):
        assert self.is_master

        # This is an algorithm similar to Hungarian algorithm - https://en.wikipedia.org/wiki/Hungarian_algorithm
        # we want to split missions between drones with "cheapest cost"
        # where cost includes "how close drone to mission start?", "is its bettery enough?"
        # and "how easy another drone will execute that mission after its current mission"
        progress = True
        while progress:
            progress = False
            missions = list(filter(lambda mission: mission.hasNextWaypoint(), self.mission_list))
            drones = list(filter(lambda drone: drone.needTask(), available_drones.values()))
            drones_on_mission = list(filter(lambda drone: drone.targetMission is not None, available_drones.values()))
            INF = 1e12
            try_to_take_far_mission_from_others_weight = 0.25
            costs_matrix = [[INF] * len(missions)] * len(drones)
            win_j, win_i, win_cost = -1, -1, INF
            for j, drone in enumerate(drones):
                for i, mission in enumerate(missions):
                    if mission.type not in drone.payload:
                        continue

                    time_to_start = drone.timeTo(*mission.getFirstWaypoint())
                    time_to_execute = mission.getTotalLength() / drone.speed
                    if mission.type == "agro" and drone.payloadAgroVolumeLeft < time_to_execute * mission.agroVolumePerSecond:
                        continue
                    _, time_to_charge = self.timeToClosestChargeStationFrom(*mission.getLastWaypoint(), charge_stations)
                    total_time = time_to_start + time_to_execute + time_to_charge
                    if total_time > drone.lifetime_left:
                        # this drone can't finish this mission part
                        continue

                    closest_mission_finish_time = INF
                    for another_drone in drones_on_mission:
                        assert another_drone.targetMission is not None
                        another_drone_time_to_finish = another_drone.targetMission.getTotalLength() / another_drone.speed
                        another_drone_time_to_start = distbetween(*another_drone.targetMission.getLastWaypoint(), *mission.getFirstWaypoint()) / another_drone.speed
                        another_drone_time_to_execute = mission.getTotalLength() / another_drone.speed
                        _, another_drone_time_to_charge = another_drone.timeToClosestChargeStationFrom(*mission.getLastWaypoint(), charge_stations)
                        another_drone_time = another_drone_time_to_finish + another_drone_time_to_start + another_drone_time_to_execute + another_drone_time_to_charge
                        another_drone_can_take_the_same_mission = another_drone_time < another_drone.lifetime_left
                        if another_drone_can_take_the_same_mission:
                            closest_mission_finish_time = min(another_drone_time_to_start, closest_mission_finish_time)

                    cost = time_to_start + time_to_execute - closest_mission_finish_time * try_to_take_far_mission_from_others_weight
                    costs_matrix[j][i] = cost
                    if cost < win_cost:
                        win_j, win_i, win_cost = j, i, cost
            if win_j != -1 and win_i != -1:
                mission = missions[win_i]
                drones[win_j].addTask(mission, world)
                self.mission_list.remove(mission)
                progress = True


def load_drones(json_path, start_x, start_y, world):
    with open(json_path, "r") as file:
        drones_data = json.load(file)
    assert "drones" in drones_data
    master_drone = None
    drones = {}
    for drone_data in drones_data["drones"]:
        drone = Drone(drone_data, start_x, start_y, world.charge_power)
        if drone.is_master:
            drone.key = str(len(drones))
            assert master_drone is None
            master_drone = drone
        else:
            key = str(len(drones))
            drone.key = key
        drones[drone.key] = drone
    assert master_drone is not None
    assert len(drones) >= 2
    print("1+{} drones loaded".format(len(drones) - 1))
    return drones
