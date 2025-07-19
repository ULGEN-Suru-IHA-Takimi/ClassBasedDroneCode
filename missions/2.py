#!/usr/bin/env python3

import asyncio
import math
from controllers.mission_controller import Mission
from controllers.xbee_controller import XBeePackage # XBeePackage doğrudan import edildi

class CircleMission(Mission):
    """
    Drone'un belirli bir merkez etrafında dairesel bir yörünge izlemesini sağlayan görev.
    Parametreler:
    - drone_ids: Bu görevi yapacak dronların ID'leri (liste)
    - center_wp_id: Dairenin merkezi olacak waypoint'in ID'si
    - radius_m: Dairenin yarıçapı (metre)
    - num_points: Daireyi oluşturacak nokta sayısı
    - altitude_m: Dairenin uçulacağı irtifa (metre)
    """
    MISSION_ID = "2"
    mission_name = "Daire Görevi"

    def __init__(self, drone_controller, mission_id: str, mission_name: str, params: dict):
        super().__init__(drone_controller, mission_id, mission_name, params)
        self.drone_ids = params.get("drone_ids", [])
        self.center_wp_id = params.get("center_wp_id")
        self.radius_m = float(params.get("radius_m", 10.0)) # Varsayılan 10 metre
        self.num_points = int(params.get("num_points", 36)) # GÜNCELLENDİ: Varsayılan 36 nokta
        self.altitude_m = float(params.get("altitude_m", 15.0)) # Varsayılan 15 metre

        if not self.center_wp_id:
            raise ValueError("Daire görevi için 'center_wp_id' parametresi zorunludur.")
        
        print(f"[{self.mission_name} - ID:{self.mission_id}]: Başlatılıyor. Merkez WP: {self.center_wp_id}, Yarıçap: {self.radius_m}m, Nokta Sayısı: {self.num_points}, İrtifa: {self.altitude_m}m")

    async def start(self):
        drone = self.drone_controller.drone
        xbee = self.drone_controller.xbee_controller
        my_drone_id = self.drone_controller.DRONE_ID

        print(f"[{self.mission_name} - ID:{self.mission_id}]: Drone {my_drone_id} göreve başlıyor: Havalanma...")
        
        # Drone'u arm et ve havalandır
        try:
            await drone.action.arm()
            print(f"[{self.mission_name} - ID:{self.mission_id}]: Drone {my_drone_id} arm edildi.")
            await asyncio.sleep(1)
            await drone.action.takeoff()
            print(f"[{self.mission_name} - ID:{self.mission_id}]: Drone {my_drone_id} havalandı.")
            await asyncio.sleep(5) 
        except Exception as e:
            print(f"[{self.mission_name} - ID:{self.mission_id}]: Havalanma hatası: {e}")
            return False

        # Görev onay paketi gönder
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
            await drone.action.land()
            return False

        # Merkez waypoint'i al
        center_waypoint = self.drone_controller.waypoint_manager.read(self.center_wp_id)
        if not center_waypoint:
            print(f"[{self.mission_name} - ID:{self.mission_id}]: Merkez waypoint '{self.center_wp_id}' bulunamadı. Görev iptal ediliyor.")
            await drone.action.land()
            return False

        # Dairesel waypointleri hesapla ve git
        print(f"[{self.mission_name} - ID:{self.mission_id}]: Dairesel yolu uçuluyor...")
        for i in range(self.num_points + 1): # Başlangıç noktasına geri dönmek için +1
            angle = 2 * math.pi * i / self.num_points
            
            # Basit Lat/Lon hesaplaması (küçük mesafeler için yeterli olabilir)
            lat_deg_per_meter = 1.0 / 111320.0
            lon_deg_per_meter = 1.0 / (111320.0 * math.cos(math.radians(center_waypoint.lat)))

            new_lat = center_waypoint.lat + self.radius_m * math.cos(angle) * lat_deg_per_meter
            new_lon = center_waypoint.lon + self.radius_m * math.sin(angle) * lon_deg_per_meter
            
            print(f"[{self.mission_name} - ID:{self.mission_id}]: Nokta {i+1}/{self.num_points+1} hedefine gidiliyor: Lat={new_lat:.6f}, Lon={new_lon:.6f}, Alt={self.altitude_m}m")
            
            # Yönü, bir sonraki noktaya doğru dinamik olarak ayarlayabiliriz.
            # Ancak basitlik için şimdilik sabit tutalım veya 0 (Kuzey) olarak bırakalım.
            # Gerçek dairesel uçuş için yönün sürekli hedefe doğru ayarlanması gerekebilir.
            await drone.action.goto_location(new_lat, new_lon, self.altitude_m, 0) # Yönü 0 olarak varsaydık
            await asyncio.sleep(1) # GÜNCELLENDİ: Her nokta arasında bekleme süresi azaltıldı

        print(f"[{self.mission_name} - ID:{self.mission_id}]: Dairesel yol tamamlandı. İniş yapılıyor...")
        await drone.action.land()
        print(f"[{self.mission_name} - ID:{self.mission_id}]: Drone {my_drone_id} iniş yaptı. Görev tamamlandı.")
        return True

