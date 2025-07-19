#!/usr/bin/env python3

import asyncio
from controllers.mission_controller import Mission

class SimpleWaypointMission(Mission):
    """
    Basit bir waypoint görev örneği.
    Parametreler:
    - drone_ids: Bu görevi yapacak dronların ID'leri (liste)
    - formation: Formasyon tipi (örn: "V", "Line")
    - waypoint_ids: Gidilecek waypoint ID'lerinin listesi
    """
    def __init__(self, drone_controller, params: dict):
        super().__init__(drone_controller, params)
        self.mission_id = "SimpleWaypointMission" # Görev ID'sini burada da belirtebiliriz
        self.drone_ids = params.get("drone_ids", [])
        self.formation = params.get("formation", "None")
        self.waypoint_ids = params.get("waypoint_ids", [])
        print(f"[SimpleWaypointMission]: Başlatılıyor. Dronlar: {self.drone_ids}, Formasyon: {self.formation}, Waypointler: {self.waypoint_ids}")

    async def start(self):
        """
        Basit görev mantığı:
        1. Drone'u havalandır.
        2. Diğer dronlardan onay bekle (sürü İHA senkronizasyonu için).
        3. Belirtilen waypoint'lere sırayla git.
        4. İniş yap.
        """
        drone = self.drone_controller.drone
        xbee = self.drone_controller.xbee_controller
        my_drone_id = self.drone_controller.DRONE_ID

        print(f"[SimpleWaypointMission]: Drone {my_drone_id} göreve başlıyor: Havalanma...")
        
        # Drone'u arm et ve havalandır
        try:
            await drone.action.arm()
            print(f"[SimpleWaypointMission]: Drone {my_drone_id} arm edildi.")
            await asyncio.sleep(1) # Güvenlik beklemesi
            await drone.action.takeoff()
            print(f"[SimpleWaypointMission]: Drone {my_drone_id} havalandı.")
            await asyncio.sleep(5) # Havalanma için bekle
        except Exception as e:
            print(f"[SimpleWaypointMission]: Havalanma hatası: {e}")
            return False

        # Görev onay paketi gönder
        confirm_package = self.drone_controller.xbee_controller.XBeePackage(
            package_type="MC",
            sender=my_drone_id,
            params={"id": self.mission_id}
        )
        xbee.send(confirm_package)
        print(f"[SimpleWaypointMission]: Görev onay paketi gönderildi: {self.mission_id}")

        # Tüm dronlardan onay bekle
        if not await self.wait_for_all_confirmations():
            print(f"[SimpleWaypointMission]: Görev başlatılamadı, tüm onaylar alınamadı.")
            await drone.action.land() # Güvenli iniş
            return False

        # Waypointlere gitme döngüsü
        for wp_id in self.waypoint_ids:
            print(f"[SimpleWaypointMission]: Waypoint {wp_id} hedefine gidiliyor...")
            if not await self.drone_controller.goto_waypoint(wp_id):
                print(f"[SimpleWaypointMission]: Waypoint {wp_id} hedefine ulaşılamadı. Görev iptal ediliyor.")
                await drone.action.land() # Güvenli iniş
                return False
            print(f"[SimpleWaypointMission]: Waypoint {wp_id} hedefine ulaşıldı.")
            await asyncio.sleep(2) # Waypointte bekleme

        print(f"[SimpleWaypointMission]: Tüm waypointlere ulaşıldı. İniş yapılıyor...")
        await drone.action.land()
        print(f"[SimpleWaypointMission]: Drone {my_drone_id} iniş yaptı. Görev tamamlandı.")
        return True