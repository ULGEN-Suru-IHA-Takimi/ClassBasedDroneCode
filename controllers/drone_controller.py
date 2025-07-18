#!/usr/bin/env python3

import asyncio
from mavsdk import System
import argparse
import time

import threading 
import sys
import os 
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from xbee_controller import XBeeController


class DroneController(XBeeController):
    def __init__(self, system_address="udpin://0.0.0.0:14540", xbee_port="/dev/ttyUSB0", drone_id=None):
        super().__init__(port=xbee_port, drone_id=drone_id)
        self.system_address = system_address
        self.drone = System()
        self.flying_alt = None
        self.waypoints = [
            (47.397606, 8.543060, None, 0),    # Waypoint 1
            (47.398106, 8.543560, None, 90),   # Waypoint 2
            (47.397106, 8.544060, None, 180),  # Waypoint 3
        ]

    async def connect(self):
        await self.drone.connect(system_address=self.system_address)
        print("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print(f"-- Connected to drone!")
                break

    async def wait_for_global_position(self):
        print("Waiting for drone to have a global position estimate...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position state is good enough for flying.")
                break

    async def get_home_altitude(self):
        print("Fetching amsl altitude at home location....")
        async for terrain_info in self.drone.telemetry.home():
            return terrain_info.absolute_altitude_m

    async def arm_and_takeoff(self, altitude=None):
        print("-- Arming")
        await self.drone.action.arm()
        print("-- Taking off")
        await self.drone.action.takeoff()
        if altitude is None:
            altitude = await self.get_home_altitude() + 10.0
        self.flying_alt = altitude
        print(f"-- Bekleniyor... (Yükseklik: {altitude}m)")
        await asyncio.sleep(5)  # 5 saniye bekle, sonra göreve başla
        # Toleranslı ve döngüsel yükseklik kontrolü
        max_wait = 30  # saniye
        waited = 0
        while waited < max_wait:
            async for position in self.drone.telemetry.position():
                print(f"Mevcut yükseklik: {position.relative_altitude_m:.2f}m")
                if position.relative_altitude_m >= self.flying_alt - 5.0:
                    print("-- Drone reached flying altitude, starting waypoint mission")
                    return
            await asyncio.sleep(1)
            waited += 1
        print("⚠️ Yükseklik beklenenden düşük, göreve başlıyor!")

    async def fly_waypoints(self):
        # Update waypoints with correct altitude
        self.waypoints = [
            (lat, lon, self.flying_alt, yaw)
            for (lat, lon, _, yaw) in self.waypoints
        ]
        try:
            for i, (lat, lon, alt, yaw) in enumerate(self.waypoints, 1):
                print(f"-- Going to waypoint {i}: ({lat}, {lon})")
                await self.drone.action.goto_location(lat, lon, alt, yaw)
                await asyncio.sleep(2)
                print(f"-- Flying to waypoint {i}...")
                target_reached = False
                while not target_reached:
                    async for position in self.drone.telemetry.position():
                        # Diğer drone'dan GPS verisini al
                        other_gps = self.get_last_received()
                        print(f"Diğer drone'dan alınan GPS: {other_gps}")
                        # Kendi GPS'ini diğer drone'lara gönder
                        my_gps = self.get_gps(position.latitude_deg, position.longitude_deg)
                        # Mesajı OUT kuyruğuna ekle
                        with self.queue_lock:
                            self.signal_queue.append((time.time(), 'OUT', my_gps))
                        lat_diff = abs(position.latitude_deg - lat)
                        lon_diff = abs(position.longitude_deg - lon)
                        if lat_diff < 0.0001 and lon_diff < 0.0001:
                            print(f"-- Reached waypoint {i}")
                            target_reached = True
                            break
                    if not target_reached:
                        await asyncio.sleep(1)
                print(f"-- Entering hold mode at waypoint {i} for 10 seconds...")
                await self.drone.action.hold()
                await asyncio.sleep(10)
                print(f"-- Finished loitering at waypoint {i}")
        finally:
            if self.device.is_open():
                self.device.close()
                print("🔌 Bağlantı kapatıldı.")
                
    async def land(self):
        print("-- All waypoints completed!")
        print("-- Landing")
        await self.drone.action.land()

    async def stay_connected(self):
        while True:
            print("Staying connected, press Ctrl-C to exit")
            await asyncio.sleep(1)

    async def run(self):
        await self.connect()
        await self.wait_for_global_position()
        await self.arm_and_takeoff()
        # XBee cihazını ve thread'leri burada başlat
        self.device.open()
        self.device.add_data_received_callback(self.data_receive_callback)
        sender_thread = threading.Thread(target=self.send_data_periodically, daemon=True)
        sender_thread.start()
        cleaner_thread = threading.Thread(target=self.queue_cleaner, daemon=True)
        cleaner_thread.start()
        await self.fly_waypoints()
        await self.land()
        await self.stay_connected()    


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Class Based Drone Controller")
    parser.add_argument("--drone_id", type=str, default="DRONE_1", help="Drone ID (for multi-drone setups)")
    parser.add_argument("--drone_port", type=str, default="udp://:14540", help="UDP address/port for drone connection (e.g. udp://:14540)")
    parser.add_argument("--xbee_port", type=str, default="/dev/ttyUSB0", help="XBee port (örn: /dev/ttyUSB0)")
    args = parser.parse_args()

    system_address = args.drone_port
    print(f"Selected Drone ID: {args.drone_id}")
    controller = DroneController(system_address=system_address, xbee_port=args.xbee_port, drone_id=args.drone_id)
    asyncio.run(controller.run())
