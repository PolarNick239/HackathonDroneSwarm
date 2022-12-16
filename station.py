import json


class Station:

    def __init__(self, station_data, key) -> None:
        self.type = station_data["type"]
        self.x = station_data["x"]
        self.y = station_data["y"]
        self.key = key


def load_stations(json_path):
    with open(json_path, "r") as file:
        stations_data = json.load(file)
    assert "stations" in stations_data
    control_station = None
    charge_stations = {}
    for ind, station_data in enumerate(stations_data["stations"]):
        key = ind + 1
        station = Station(station_data, key)
        if station.type == "control":
            assert control_station is None
            control_station = station
        else:
            assert station.type == "charge"
            charge_stations[len(charge_stations) + 1] = station
    assert control_station is not None
    assert len(charge_stations) >= 1
    print("1+{} stations loaded".format(len(charge_stations)))
    return control_station, charge_stations
