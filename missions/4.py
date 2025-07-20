#!/usr/bin/env python3

import asyncio
from mavsdk import System
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from controllers.drone_controller import DroneController


async def run():
    drone_controller = DroneController(sys_address="udpin://0.0.0.0:14540", xbee_port="/dev/ttyUSB0")
    await drone_controller.connect()

    print("Waiting for drone to connect...")
    async for state in drone_controller.drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone_controller.drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position state is good enough for flying.")
            break

    print("Fetching amsl altitude at home location....")
    async for terrain_info in drone_controller.drone.telemetry.home():
        absolute_altitude = terrain_info.absolute_altitude_m
        break

    print("-- Arming")
    await drone_controller.drone.action.arm()

    print("-- Taking off")
    await drone_controller.drone.action.takeoff()

    flying_alt = absolute_altitude + 20.0
    print("-- Waiting for drone to reach flying altitude...")
    while True:
        async for position in drone_controller.drone.telemetry.position():
            if position.relative_altitude_m >= 10.0:
                print("-- Drone reached flying altitude, starting waypoint mission")
                break
        break
    await asyncio.sleep(2)

    waypoints = [
        (47.397606, 8.543060, flying_alt, 0),
        (47.398106, 8.543560, flying_alt, 90),
        (47.397106, 8.544060, flying_alt, 180),
    ]

    for i, (lat, lon, alt, yaw) in enumerate(waypoints, 1):
        print(f"-- Going to waypoint {i}: ({lat}, {lon})")
        await drone_controller.goto_location_horizontal_only(lat, lon, flying_alt, yaw)
        print(f"-- Reached waypoint {i}")
        print(f"-- Entering hold mode at waypoint {i} for 10 seconds...")
        await drone_controller.drone.action.hold()
        await asyncio.sleep(10)
        print(f"-- Finished loitering at waypoint {i}")

    print("-- All waypoints completed!")
    print("-- Landing")
    await drone_controller.drone.action.land()

    while True:
        print("Staying connected, press Ctrl-C to exit")
        await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(run())
