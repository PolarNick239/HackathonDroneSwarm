## world.json

 - dem_resolution: meters/pixel
 - maximum_allowed_height: any intensity in DEM bigger than maximum_allowed_height is prohibited
 - simulation_step: time of each step (in seconds)
 - wireless_range: meters (DJI P4: 7 km video transmission range)
 - charge_time: seconds (5 minutes ~= 300 seconds, we use a lot of pre-charged batteries)

## drones.json

Possible payloads:

 - agro
 - 3dLidar
 - highresCamera
 - gimbolCamera
 - cargo
 - geoRadar
 - speed: m/s (30 km/h ~= 8.3 m/s)
 - lifetime: seconds (40 minutes - 10 minutes to be safe = 30 minutes ~= 1800 seconds)

## tasks.json

Possible tasks:

 - agro
 - 3dLidar
 - highresCamera
 - gimbolCamera
 - cargo
 - geoRadar


