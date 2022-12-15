import json


class Drone:

    def __init__(self, drone_data, start_x, start_y, speed, max_lifetime):
        self.is_master = drone_data["isMaster"]
        self.payload = drone_data["payload"]
        self.x = start_x
        self.y = start_y

        self.targetX = None
        self.targetY = None

        self.max_lifetime = max_lifetime
        self.lifetime_left = max_lifetime

        self.state = "..."

    def fly(self):
        if self.targetY is None:
            return

    def checkIfBatteryIsLow(self):
        pass # TODO

    def addTask(self, task):
        pass # TODO

    def tryToScheduleTask(self, drones):
        pass # TODO


def load_drones(json_path, start_x, start_y, world):
    with open(json_path, "r") as file:
        drones_data = json.load(file)
    assert "drones" in drones_data
    master_drone = None
    slave_drones = {}
    for drone_data in drones_data["drones"]:
        drone = Drone(drone_data, start_x, start_y, world.drones_speed, world.drone_lifetime)
        if drone.is_master:
            assert master_drone is None
            master_drone = drone
        else:
            slave_drones[len(slave_drones) + 1] = drone
    assert master_drone is not None
    assert len(slave_drones) >= 1
    print("1+{} drones loaded".format(len(slave_drones)))
    return master_drone, slave_drones
