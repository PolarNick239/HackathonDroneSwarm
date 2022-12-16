import json
import random

from mission import Mission
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

        self.state = "wait"

    def distanceTo(self, x, y):
        return dist(x - self.x, y - self.y)

    def timeTo(self, x, y):
        distance = self.distanceTo(x, y)
        return distance / self.speed

    def speedLimit(self, vx, vy):
        speed = max(1, dist(vx, vy))
        ratio = self.speed / speed
        return vx * ratio, vy * ratio

    def fly(self, dt):
        if self.targetX is None:
            return
        vx, vy = self.speedLimit(self.targetX - self.x, self.targetY - self.y)
        vx, vy = dt * vx, dt * vy
        distanceToTarget = self.distanceTo(self.targetX, self.targetY)
        if dist(vx, vy) < distanceToTarget:
            self.x += vx
            self.y += vy
        else:
            self.x, self.y = self.targetX, self.targetY
            self.targetX, self.targetY = None, None
            if self.state == "flyToMission":
                print("Drone {}: executing mission {}...".format(self.key, self.targetMission.key))
                self.state = "onMission"
            elif self.state == "flyToCharge":
                print("Drone {}: on charge...".format(self.key))
                self.state = "onCharge"
            else:
                raise Exception("fly(): state={} is incorrect!".format(self.state))

    def updateMission(self, dt):
        assert self.state == "onMission"
        self.targetMission.time_to_finish_left = max(0, self.targetMission.time_to_finish_left - dt)
        if self.targetMission.time_to_finish_left == 0:
            print("Drone {}: mission {} finished".format(self.key, self.targetMission.key))
            self.targetMission = None
            self.state = "wait"

    def updateCharge(self, dt):
        assert self.state == "onCharge"
        self.lifetime_left = min(self.max_lifetime, self.lifetime_left + self.charge_power * dt)
        if self.lifetime_left == self.max_lifetime:
            print("Drone {}: charge finished".format(self.key))
            if self.targetMission is None:
                print("Drone {}: charged! waiting...".format(self.key))
                self.state = "wait"
            else:
                print("Drone {}: charged! continue mission {}".format(self.key, self.targetMission.key))
                self.state = "flyToMission"
                self.targetX = self.targetMission.x
                self.targetY = self.targetMission.y

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
            #TODO make task as well?
            self.targetX, self.targetY = closest_station.x, closest_station.y

    def update(self, world, dt):
        if self.state not in {"flyToCharge", "onCharge"}:
            self.checkIfBatteryIsLow(world.charge_stations)

        # print('update: drone state: {}'.format(self.state))

        if self.state in {"flyToMission", "flyToCharge"}:
            self.fly(dt)
        elif self.state == "onMission":
            self.updateMission(dt)
        elif self.state == "onCharge":
            self.updateCharge(dt)
        elif self.state == "wait":
            pass  # TODO we can fly into the center of area (or closer to the charge station/go to charge if we have less than 50% battery)
        else:
            raise Exception("state={} is incorrect!".format(self.state))
        self.lifetime_left -= dt

    def needTask(self):
        return self.targetMission is None and self.state in {"wait", "flyToCharge", "onCharge"}

    def addTask(self, mission: Mission):
        assert self.needTask()

        self.targetMission = mission
        if self.state in {"wait"}:
            self.state = "flyToMission"
            self.targetX = mission.x
            self.targetY = mission.y

    def tryToScheduleTask(self, drones):
        assert self.is_master
        for key, drone in drones.items():
            if not drone.needTask():
                continue
            mission = Mission(key, 1000, random.random() * 22500, random.random() * 22500)
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
            drone.key = "Master"
            assert master_drone is None
            master_drone = drone
        else:
            key = str(len(drones))
            drone.key = key
        drones[drone.key] = drone
    assert master_drone is not None
    assert len(drones) >= 2
    print("1+{} drones loaded".format(len(drones) - 1))
    return master_drone, drones
