#!/usr/bin/env python3

import asyncio
from controllers.mission_controller import Mission
from controllers.xbee_controller import XBeePackage # XBeePackage doğrudan import edildi

class SimpleWaypointMission(Mission):

    MISSION_ID = "0"
    mission_name = "Hello World"

    def __init__(self, drone_controller, mission_id: str, mission_name: str, params: dict):
        super().__init__(drone_controller, mission_id, mission_name, params)

        self.variable = params.get("variable", "Hello World")
        self.wp_id = params.get("wp","1")


        print(f"[{self.mission_name} - ID:{self.mission_id}]: Başlatılıyor. Dronlar: {self.drone_ids}, Waypoint: {self.waypoint}, Varsayılan İrtifa: {self.default_altitude_m}m")

    async def start(self):

        drone = self.drone_controller.drone
        xbee = self.drone_controller.xbee_controller

        my_drone_id = self.drone_controller.DRONE_ID

        print(self.variable)

        # Drone'u arm et ve havalandır
        try:
            await drone.action.arm()
            print(f"[{self.mission_name} - ID:{self.mission_id}]: Drone {my_drone_id} arm edildi.")
            await asyncio.sleep(1) # Güvenlik beklemesi
            await drone.action.takeoff()
            print(f"[{self.mission_name} - ID:{self.mission_id}]: Drone {my_drone_id} havalandı.")
            await asyncio.sleep(5) # Havalanma için bekle
        except Exception as e:
            print(f"[{self.mission_name} - ID:{self.mission_id}]: Havalanma hatası: {e}")
            return False


        # Görev onay paketi gönder
        confirm_package = XBeePackage( 
            package_type="MC",
            sender=self.drone_controller.DRONE_ID,
            params={"id": self.mission_id}
        )
        xbee.send(confirm_package)
        print(f"[{self.mission_name} - ID:{self.mission_id}]: Görev onay paketi gönderildi: {self.mission_id}")

        # Waypoint'i doğrudan bir tuple olarak tanımlamak yerine, Waypoint nesnesi gibi erişilebilir hale getirelim
        # Veya doğrudan goto_location'a lat, lon, alt, hed geçirelim
        # Waypoint'i tuple olarak tanımladığınız için, elemanlarına tek tek erişmeliyiz.
        waypoint_coords = (47.397606, 8.543060, 10.0, 0) # (lat, lon, alt, hed)
        
        # Waypoint'in kendi irtifası varsa onu kullan, yoksa varsayılanı kullan
        # Bu satırda waypoint bir tuple olduğu için, waypoint[2] kullanıldı.
        target_altitude = waypoint_coords[2] if waypoint_coords[2] is not None else self.default_altitude_m

        print(f"[{self.mission_name} - ID:{self.mission_id}]: Koordinat hedefine gidiliyor: Lat={waypoint_coords[0]:.6f}, Lon={waypoint_coords[1]:.6f}, Alt={target_altitude}m, Hed={waypoint_coords[3]} derece")

        # goto_location metoduna ayrı ayrı lat, lon, alt, hed değerlerini geçirin
        if not await self.drone_controller.goto_location(waypoint_coords[0], waypoint_coords[1], target_altitude, waypoint_coords[3]):
            print(f"[{self.mission_name} - ID:{self.mission_id}]: Koordinat hedefine ulaşılamadı. Görev iptal ediliyor.")
            await drone.action.land() # Güvenli iniş
            return False
        print(f"[{self.mission_name} - ID:{self.mission_id}]: Koordinat hedefine ulaşıldı.")
        await asyncio.sleep(2) # Waypointte bekleme

        print(f"[{self.mission_name} - ID:{self.mission_id}]: Tüm waypointlere ulaşıldı. İniş yapılıyor...")
        await drone.action.land()
        print(f"[{self.mission_name} - ID:{self.mission_id}]: Drone {self.drone_controller.DRONE_ID} iniş yaptı. Görev tamamlandı.")
        return True

