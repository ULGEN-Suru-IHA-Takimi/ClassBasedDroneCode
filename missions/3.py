#!/usr/bin/env python3

import asyncio
import math
from controllers.mission_controller import Mission
from controllers.xbee_controller import XBeePackage # XBeePackage doğrudan import edildi

class SearchPatternMission(Mission):
    """
    Drone'un belirli bir başlangıç noktasından başlayarak kare şeklinde bir arama deseni uçmasını sağlayan görev.
    Bu güncellenmiş versiyon, daha dinamik yönlendirme ve geliştirilmiş loglama içerir.
    Parametreler:
    - drone_ids: Bu görevi yapacak dronların ID'leri (liste)
    - start_wp_id: Arama deseninin başlangıç noktası olacak waypoint'in ID'si
    - side_length_m: Karenin bir kenar uzunluğu (metre)
    - altitude_m: Arama deseninin uçulacağı irtifa (metre)
    - num_laps: Desenin kaç kez tekrarlanacağı
    - offset_m: Sürü İHA'lar için desene eklenecek yanal ofset (metre). Tek drone için 0.
    """
    MISSION_ID = "3"
    mission_name = "Arama Deseni Görevi (Güncellenmiş)"

    def __init__(self, drone_controller, mission_id: str, mission_name: str, params: dict):
        super().__init__(drone_controller, mission_id, mission_name, params)
        self.drone_ids = params.get("drone_ids", [])
        self.start_wp_id = params.get("start_wp_id")
        self.side_length_m = float(params.get("side_length_m", 50.0)) # Varsayılan 50 metre
        self.altitude_m = float(params.get("altitude_m", 20.0)) # Varsayılan 20 metre
        self.num_laps = int(params.get("num_laps", 1)) # Varsayılan 1 tur
        self.offset_m = float(params.get("offset_m", 0.0)) # Yeni: Yanal ofset (sürü için)

        if not self.start_wp_id:
            raise ValueError("Arama deseni görevi için 'start_wp_id' parametresi zorunludur.")

        print(f"[{self.mission_name} - ID:{self.mission_id}]: Başlatılıyor. Başlangıç WP: {self.start_wp_id}, Kenar Uzunluğu: {self.side_length_m}m, İrtifa: {self.altitude_m}m, Tur Sayısı: {self.num_laps}, Yanal Ofset: {self.offset_m}m")

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
            await drone.action.land()
            return False

        # Başlangıç waypoint'i al
        start_waypoint = self.drone_controller.waypoint_manager.read(self.start_wp_id)
        if not start_waypoint:
            print(f"[{self.mission_name} - ID:{self.mission_id}]: Başlangıç waypoint '{self.start_wp_id}' bulunamadı. Görev iptal ediliyor.")
            await drone.action.land()
            return False

        current_lat = start_waypoint.lat
        current_lon = start_waypoint.lon

        # Kare arama deseni uç
        print(f"[{self.mission_name} - ID:{self.mission_id}]: Kare arama deseni uçuluyor...")
        
        # Metre cinsinden enlem ve boylam değişimini dereceye çevirmek için yaklaşık değerler
        lat_deg_per_meter = 1.0 / 111320.0
        lon_deg_per_meter = 1.0 / (111320.0 * math.cos(math.radians(current_lat)))

        # Başlangıç noktasına ofset uygula (eğer varsa)
        current_lon += self.offset_m * lon_deg_per_meter # Yanal ofset

        for lap in range(self.num_laps):
            print(f"[{self.mission_name} - ID:{self.mission_id}]: Tur {lap + 1}/{self.num_laps} başlıyor.")

            # Sağ (Doğuya)
            target_lat = current_lat
            target_lon = current_lon + self.side_length_m * lon_deg_per_meter
            print(f"[{self.mission_name} - ID:{self.mission_id}]: Hedef: Sağ (Lat: {target_lat:.6f}, Lon: {target_lon:.6f})")
            await drone.action.goto_location(target_lat, target_lon, self.altitude_m, 90) # Yönü Doğu (90 derece)
            await asyncio.sleep(2)
            current_lat, current_lon = target_lat, target_lon

            # Yukarı (Kuzeye)
            target_lat = current_lat + self.side_length_m * lat_deg_per_meter
            target_lon = current_lon
            print(f"[{self.mission_name} - ID:{self.mission_id}]: Hedef: Yukarı (Lat: {target_lat:.6f}, Lon: {target_lon:.6f})")
            await drone.action.goto_location(target_lat, target_lon, self.altitude_m, 0) # Yönü Kuzey (0 derece)
            await asyncio.sleep(2)
            current_lat, current_lon = target_lat, current_lon

            # Sol (Batıya)
            target_lat = current_lat
            target_lon = current_lon - self.side_length_m * lon_deg_per_meter
            print(f"[{self.mission_name} - ID:{self.mission_id}]: Hedef: Sol (Lat: {target_lat:.6f}, Lon: {target_lon:.6f})")
            await drone.action.goto_location(target_lat, target_lon, self.altitude_m, -90) # Yönü Batı (-90 veya 270 derece)
            await asyncio.sleep(2)
            current_lat, current_lon = target_lat, current_lon

            # Aşağı (Güneye - başlangıç noktasına geri)
            target_lat = current_lat - self.side_length_m * lat_deg_per_meter
            target_lon = current_lon
            print(f"[{self.mission_name} - ID:{self.mission_id}]: Hedef: Aşağı (Lat: {target_lat:.6f}, Lon: {target_lon:.6f})")
            await drone.action.goto_location(target_lat, target_lon, self.altitude_m, 180) # Yönü Güney (180 derece)
            await asyncio.sleep(2)
            current_lat, current_lon = target_lat, current_lon

        print(f"[{self.mission_name} - ID:{self.mission_id}]: Arama deseni tamamlandı. İniş yapılıyor...")
        await drone.action.land()
        print(f"[{self.mission_name} - ID:{self.mission_id}]: Drone {my_drone_id} iniş yaptı. Görev tamamlandı.")
        return True

