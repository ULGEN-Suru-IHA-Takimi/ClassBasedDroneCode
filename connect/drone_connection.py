#!/usr/bin/env python3

import asyncio
from mavsdk import System

class DroneConnection:
    def __init__(self, sys_address: str = "udpin://0.0.0.0:14540"):
        self.sys_address = sys_address
        self.drone = System()
    
    async def connect(self) -> None:
        print(f"Connecting to drone at {self.sys_address}")
        await self.drone.connect(system_address=self.sys_address)

        # Status text task'i başlat ama hataları yakala
        status_text_task = asyncio.create_task(self.print_status_text(self.drone))
        
        print("Waiting for drone to connect...")
        
        # Timeout ile bağlantı kontrolü
        try:
            await asyncio.wait_for(self._wait_for_connection(), timeout=30.0)
        except asyncio.TimeoutError:
            print("Connection timeout! Make sure PX4 SITL is running.")
            status_text_task.cancel()
            raise
        
        # Status task'i iptal et
        status_text_task.cancel()
        try:
            await status_text_task
        except asyncio.CancelledError:
            pass

    async def _wait_for_connection(self):
        """Wait for drone connection and global position"""
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("-- Connected to drone!")
                break
        
        print("Waiting for drone to have a global position estimate...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position estimate OK")
                break

    async def print_status_text(self, drone) -> None:
        try:
            async for status_text in drone.telemetry.status_text():
                print(f"Status: {status_text.type}: {status_text.text}")
        except asyncio.CancelledError:
            pass  # Normal iptal
        except Exception as e:
            print(f"Status text error (this is normal): {e}")
            pass  # Bağlantı koptuğunda normal

            

async def run(sys_address = "udpin://0.0.0.0:14540"):
    drone_connection = DroneConnection(sys_address)
    await drone_connection.connect()

if __name__ == "__main__":
    asyncio.run(run())