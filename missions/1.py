#!/usr/bin/env python3

import asyncio
from controllers.mission_controller import Mission
from controllers.xbee_controller import XBeePackage # XBeePackage doğrudan import edildi

class SimpleWaypointMission(Mission):
    """
    Basit bir waypoint görev örneği.
    Bu güncellenmiş versiyon, daha detaylı loglama ve parametre yönetimi içerir.
    Parametreler:
    - drone_ids: Bu görevi yapacak dronların ID'leri (liste)
    - waypoint_ids: Gidilecek waypoint ID'lerinin listesi
    - altitude_m: Waypointlere gidilirken kullanılacak varsayılan irtifa (metre).
                  Eğer waypoint'in kendi irtifası varsa o kullanılır, yoksa bu varsayılan kullanılır.
    """
    MISSION_ID = "1"
    mission_name = "Basit Waypoint Görevi (Güncellenmiş)"

    def __init__(self, drone_controller, mission_id: str, mission_name: str, params: dict):
        super().__init__(drone_controller, mission_id, mission_name, params)
        
        self.drone_ids = params.get("drone_ids", [])
        self.waypoint_ids = params.get("waypoint_ids", [])
        self.default_altitude_m = float(params.get("altitude_m", 10.0)) # Varsayılan 10 metre

        if not self.waypoint_ids:
            raise ValueError("Basit Waypoint görevi için 'waypoint_ids' parametresi zorunludur.")
        
        print(f"[{self.mission_name} - ID:{self.mission_id}]: Başlatılıyor. Dronlar: {self.drone_ids}, Waypointler: {self.waypoint_ids}, Varsayılan İrtifa: {self.default_altitude_m}m")

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

        print(f"[{self.mission_name} - ID:{self.mission_id}]: Drone {my_drone_id} göreve başlıyor: Havalanma...")
        
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
        # Düzeltme: XBeePackage doğrudan import edildiği için 'xbee_controller' ön eki kaldırıldı.
        confirm_package = XBeePackage( 
            package_type="MC",
            sender=my_drone_id,
            params={"id": self.mission_id}
        )
        xbee.send(confirm_package)
        print(f"[{self.mission_name} - ID:{self.mission_id}]: Görev onay paketi gönderildi: {self.mission_id}")

        # Tüm dronlardan onay bekle
        if not await self.wait_for_all_confirmations():
            print(f"[{self.mission_name} - ID:{self.mission_id}]: Görev başlatılamadı, tüm onaylar alınamadı.")
            await drone.action.land() # Güvenli iniş
            return False

        # Waypointlere gitme döngüsü
        for wp_id in self.waypoint_ids:
            waypoint = self.drone_controller.waypoint_manager.read(wp_id)
            if not waypoint:
                print(f"[{self.mission_name} - ID:{self.mission_id}]: Waypoint '{wp_id}' bulunamadı. Bu waypoint atlanıyor.")
                continue # Waypoint bulunamazsa atla ve bir sonrakine geç

            # Waypoint'in kendi irtifası varsa onu kullan, yoksa varsayılanı kullan
            target_altitude = waypoint.alt if waypoint.alt is not None else self.default_altitude_m

            print(f"[{self.mission_name} - ID:{self.mission_id}]: Waypoint {wp_id} hedefine gidiliyor: Lat={waypoint.lat:.6f}, Lon={waypoint.lon:.6f}, Alt={target_altitude}m, Hed={waypoint.hed} derece")
            
            if not await self.drone_controller.goto_waypoint(wp_id):
                print(f"[{self.mission_name} - ID:{self.mission_id}]: Waypoint {wp_id} hedefine ulaşılamadı. Görev iptal ediliyor.")
                await drone.action.land() # Güvenli iniş
                return False
            print(f"[{self.mission_name} - ID:{self.mission_id}]: Waypoint {wp_id} hedefine ulaşıldı.")
            await asyncio.sleep(2) # Waypointte bekleme

        print(f"[{self.mission_name} - ID:{self.mission_id}]: Tüm waypointlere ulaşıldı. İniş yapılıyor...")
        await drone.action.land()
        print(f"[{self.mission_name} - ID:{self.mission_id}]: Drone {my_drone_id} iniş yaptı. Görev tamamlandı.")
        return True

