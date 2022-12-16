import json
import random

from mission import Mission, MissionPoly
from utils import *


class Drone:

    def __init__(self, drone_data, start_x, start_y, speed, max_lifetime, charge_power):
        self.is_master = drone_data["isMaster"]
        self.payload = drone_data["payload"]
        self.x = start_x
        self.y = start_y
        self.key = None

        self.targetX = None  #TODO replace with values from mission
        self.targetY = None  #TODO replace with values from mission
        self.targetMission = None

        self.speed = speed
        self.max_lifetime = max_lifetime
        self.lifetime_left = max_lifetime
        self.charge_power = charge_power
        self.mission_list = []
        self.flying = False

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
        if self.targetMission.finished():
            print("Drone {}: mission {} finished".format(self.key, self.targetMission.key))
            self.targetMission = None
            self.state = "wait"
        elif self.targetMission.hasNextWaypoint():
            print("Drone {}: mission {} going to next waypoint".format(self.key, self.targetMission.key))
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

    def checkIfBatteryIsLow(self, charge_stations):
        closest_station = None
        smallest_time_to_reach = None
        for key, station in charge_stations.items():
            time_to_reach = self.timeTo(station.x, station.y)
            if smallest_time_to_reach is None or time_to_reach < smallest_time_to_reach:
                closest_station = station
                smallest_time_to_reach = time_to_reach
        if self.lifetime_left < 2 * smallest_time_to_reach:
            print("Drone {}: low battery, flying to station {}".format(self.key, closest_station.key))
            self.state = "flyToCharge"
            if self.targetMission is not None:
                self.mission_list.append(self.targetMission)
                self.targetMission = None
            #TODO make task as well?
            self.targetX, self.targetY = closest_station.x, closest_station.y

    def update(self, world, dt):
        if self.state not in {"flyToCharge", "onCharge"}:
            self.checkIfBatteryIsLow(world.charge_stations)

        # print('Drone {}: update: drone state: {}, target mission: {}'.format(self.key, self.state, self.targetMission))

        if self.state in {"flyToMission", "flyToCharge"}:
            self.fly(world, dt)
        elif self.state == "onMission":
            self.updateMission(dt)
        elif self.state == "onCharge":
            self.updateCharge(dt)
        elif self.state == "wait":
            pass  # TODO we can fly into the center of area (or closer to the charge station/go to charge if we have less than 50% battery)
        else:
            raise Exception("state={} is incorrect!".format(self.state))

        if self.flying:
            self.lifetime_left -= dt

    def needTask(self):
        return self.targetMission is None and self.state in {"wait"}

    def addTask(self, mission: MissionPoly):
        assert self.needTask()

        self.flying = True
        self.targetMission = mission
        if self.state in {"wait"}:
            self.state = "flyToMission"
            self.targetX = mission.nextWaypoint()[0]
            self.targetY = mission.nextWaypoint()[1]

    def selectBestMission(self, drone):
        dist = None
        result = None
        for mission in self.mission_list:
            # TODO mission type check
            if dist is None or distbetween(mission.x, mission.y, drone.x, drone.y) < dist:
                result = mission

        if result is not None:
            self.mission_list.remove(result)

        return result

    def tryToScheduleTask(self, drones):
        assert self.is_master
        for key, drone in drones.items():
            if not drone.needTask():
                continue
            # mission = Mission(key, 1000, random.random() * 22500, random.random() * 22500)
            mission = self.selectBestMission(drone)
            if mission is None:
                continue
            drone.addTask(mission)


def load_drones(json_path, start_x, start_y, world):
    with open(json_path, "r") as file:
        drones_data = json.load(file)
    assert "drones" in drones_data
    master_drone = None
    drones = {}
    for drone_data in drones_data["drones"]:
        drone = Drone(drone_data, start_x, start_y, world.drones_speed, world.drone_lifetime, world.charge_power)
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
