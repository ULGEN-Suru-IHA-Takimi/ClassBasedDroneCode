#!/usr/bin/env python3

import asyncio
import threading
import time
import json
import os
import importlib.util
from mavsdk import System, telemetry, offboard, action

# Kendi modüllerimizi import ediyoruz
# Proje kök dizininden import etmek için sys.path'e ekleme yapılabilir
# Ancak modüler yapıda olduğu için göreceli importlar tercih edilir.
# Bu dosya 'controllers' içinde olduğundan, 'connect' ve 'controllers' aynı seviyede.
from connect.drone_connection import DroneConnection
from controllers.xbee_controller import XBeeController, XBeePackage
from controllers.waypoint_controller import waypoints, Waypoint

# Missions klasöründen görevleri dinamik yüklemek için kullanılacak
# from missions.mission_controller import Mission # Bu dosya henüz oluşturulmadı, sonra eklenecek

# PID-APF için basit bir placeholder (gerçek implementasyon çok daha karmaşıktır)
# Çarpışma önleme için daha gelişmiş algoritmalar ve sensör verisi gereklidir.
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def calculate(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class APF: # Artificial Potential Field (Yapay Potansiyel Alan)
    def __init__(self, drone_id: str):
        self.drone_id = drone_id
        # Çarpışma önleme için diğer dronların konumları burada tutulabilir
        self.other_drone_positions = {} # {drone_id: (lat, lon)}

    def update_other_drone_position(self, drone_id: str, lat: float, lon: float):
        if drone_id != self.drone_id: # Kendi konumumuzu takip etmiyoruz
            self.other_drone_positions[drone_id] = (lat, lon)
            # print(f"APF: Drone {drone_id} konumu güncellendi: {lat}, {lon}")

    def calculate_avoidance_vector(self, current_lat, current_lon, target_lat, target_lon):
        # Basit bir APF mantığı: Diğer dronlara çok yakınsa itme kuvveti uygula
        # Gerçek bir APF, çekme ve itme kuvvetlerini hesaplar.
        avoidance_lat_force = 0.0
        avoidance_lon_force = 0.0
        
        # Bu kısım, diğer dronların konumlarını kullanarak çarpışma önleme vektörlerini hesaplar.
        # Gerçek bir uygulamada, dronların hızları, yönleri ve sensör verileri de dikkate alınır.
        # Örneğin, her bir diğer drone için bir itme kuvveti hesaplanabilir ve toplanabilir.
        
        # Örnek: Diğer dronlara 10 metreden yakınsa basit bir itme kuvveti uygula
        # Bu sadece bir placeholder'dır ve gerçek bir çarpışma önleme algoritması değildir.
        # Coğrafi koordinatlar arası mesafe hesaplaması (haversine formülü vb.) gereklidir.
        # Basitlik adına, şimdilik sadece konsepti gösteriyoruz.
        
        # for other_id, (other_lat, other_lon) in self.other_drone_positions.items():
        #     distance = calculate_distance(current_lat, current_lon, other_lat, other_lon)
        #     if distance < 10: # 10 metreden yakınsa
        #         # Basit bir itme kuvveti
        #         force_magnitude = (10 - distance) * 0.1 # Mesafe azaldıkça kuvvet artsın
        #         angle = calculate_angle(current_lat, current_lon, other_lat, other_lon)
        #         avoidance_lat_force -= force_magnitude * math.cos(angle)
        #         avoidance_lon_force -= force_magnitude * math.sin(angle)

        return avoidance_lat_force, avoidance_lon_force


class DroneController(DroneConnection):
    """
    DroneConnection'dan türeyen ana drone kontrol sınıfı.
    XBee iletişimi, waypoint yönetimi ve görev yürütme yeteneklerini içerir.
    """
    DRONE_ID = "1" # Bu dronun benzersiz ID'si

    def __init__(self, sys_address: str = "udpin://0.0.0.0:14540", xbee_port: str = "/dev/ttyUSB0"):
        super().__init__(sys_address)
        self.xbee_port = xbee_port
        self.xbee_controller: XBeeController = None
        self.waypoint_manager = waypoints() # Waypoint'leri yönetecek sınıf
        self.current_mission = None # Aktif görevi tutar
        self.missions = {} # Yüklenen görev sınıflarını tutar {mission_id: MissionClass}

        self._gps_send_task = None
        self._xbee_receive_task = None
        self._drone_armed_event = asyncio.Event() # Drone'un armed olup olmadığını takip eder
        self._stop_tasks_event = asyncio.Event() # Tüm async görevleri durdurmak için

        self.apf_controller = APF(self.DRONE_ID) # Çarpışma önleme için APF

        # Görev onayları için bir set (sürü İHA'lar için)
        self.mission_confirmations = set() 
        self.required_confirmations = 0 # Görev için beklenen onay sayısı

        # PID kontrolörleri (örnek değerler, ayarlanması gerekir)
        self.pid_lat = PIDController(kp=0.05, ki=0.001, kd=0.01)
        self.pid_lon = PIDController(kp=0.05, ki=0.001, kd=0.01)
        self.pid_alt = PIDController(kp=0.1, ki=0.002, kd=0.02)


    async def connect(self) -> None:
        """
        Drone'a bağlanır ve XBee iletişimini başlatır.
        """
        await super().connect() # DroneConnection'ın connect metodunu çağır

        # XBee Controller'ı başlat
        print(f"[DroneController]: XBee cihazı {self.xbee_port} üzerinden başlatılıyor...")
        self.xbee_controller = XBeeController(self.xbee_port)
        if not self.xbee_controller.connected:
            print("[DroneController]: XBee cihaza bağlanılamadı. XBee iletişimi devre dışı.")
            # XBee bağlantısı başarısız olursa diğer XBee bağımlı görevleri başlatma
            return
        
        print("[DroneController]: XBee bağlantısı başarılı. Veri dinleme başlatılıyor.")
        # XBee'den gelen verileri dinleyen asenkron görevi başlat
        self._xbee_receive_task = asyncio.create_task(self._xbee_receive_loop())

        # Drone'un armed durumunu takip eden görevi başlat
        asyncio.create_task(self._monitor_armed_state())

        # Görevleri yükle
        self.load_missions()
        
        print("[DroneController]: Drone Controller hazır.")


    async def disconnect(self) -> None:
        """
        Drone ve XBee bağlantılarını kapatır, tüm asenkron görevleri durdurur.
        """
        print("[DroneController]: Drone Controller kapatılıyor...")
        self._stop_tasks_event.set() # Tüm async görevlere durma sinyali gönder

        # Tüm async görevlerin tamamlanmasını bekle
        if self._gps_send_task and not self._gps_send_task.done():
            self._gps_send_task.cancel()
            try: await self._gps_send_task
            except asyncio.CancelledError: pass

        if self._xbee_receive_task and not self._xbee_receive_task.done():
            self._xbee_receive_task.cancel()
            try: await self._xbee_receive_task
            except asyncio.CancelledError: pass
        
        # XBee Controller'ı kapat
        if self.xbee_controller:
            self.xbee_controller.disconnect()
        
        # MAVSDK bağlantısını kapat (isteğe bağlı, System objesi kapatma metodu yok)
        # self.drone.close() # MAVSDK'da doğrudan bir close metodu yok, bağlantı kendiliğinden kapanır.
        print("[DroneController]: Drone Controller kapatıldı.")


    async def _monitor_armed_state(self) -> None:
        """Drone'un armed durumunu izler ve GPS gönderme görevini başlatır/durdurur."""
        print("[DroneController]: Drone armed durumu izleniyor...")
        async for is_armed in self.drone.telemetry.armed():
            if self._stop_tasks_event.is_set():
                break # Kapatma sinyali gelirse döngüden çık

            if is_armed and not self._drone_armed_event.is_set():
                print("[DroneController]: Drone armed edildi. GPS veri gönderimi başlatılıyor.")
                self._drone_armed_event.set()
                if not self._gps_send_task or self._gps_send_task.done():
                    self._gps_send_task = asyncio.create_task(self._send_gps_data_loop())
            elif not is_armed and self._drone_armed_event.is_set():
                print("[DroneController]: Drone disarm edildi. GPS veri gönderimi durduruluyor.")
                self._drone_armed_event.clear()
                if self._gps_send_task and not self._gps_send_task.done():
                    self._gps_send_task.cancel()
                    try: await self._gps_send_task
                    except asyncio.CancelledError: pass
        print("[DroneController]: Armed durumu izleme görevi durduruldu.")


    async def _send_gps_data_loop(self) -> None:
        """
        Drone armed olduğunda GPS verisini XBee üzerinden gönderir.
        """
        print("[DroneController]: GPS veri gönderim döngüsü başlatıldı.")
        try:
            async for position in self.drone.telemetry.position():
                if self._stop_tasks_event.is_set() or not self._drone_armed_event.is_set():
                    break # Kapatma veya disarm sinyali gelirse döngüden çık

                lat = position.latitude_deg
                lon = position.longitude_deg
                
                # XBeePackage formatına uygun hale getiriyoruz (int * 1000000)
                gps_package = XBeePackage(
                    package_type="G",
                    sender=self.DRONE_ID,
                    params={
                        "x": int(lat * 1000000),
                        "y": int(lon * 1000000)
                    }
                )
                self.xbee_controller.send(gps_package)
                await asyncio.sleep(1) # Her saniyede bir gönder
        except asyncio.CancelledError:
            print("[DroneController]: GPS veri gönderim döngüsü iptal edildi.")
        except Exception as e:
            print(f"[DroneController]: GPS veri gönderim döngüsünde hata: {e}")
        print("[DroneController]: GPS veri gönderim döngüsü durduruldu.")


    async def _xbee_receive_loop(self) -> None:
        """
        XBee'den gelen paketleri sürekli dinler ve işler.
        """
        print("[DroneController]: XBee veri alma döngüsü başlatıldı.")
        try:
            while not self._stop_tasks_event.is_set() and self.xbee_controller.connected:
                incoming_package_json = self.xbee_controller.receive()
                if incoming_package_json:
                    await self._process_xbee_package(incoming_package_json)
                else:
                    await asyncio.sleep(0.01) # CPU'yu boşa harcamamak için kısa bekleme
        except asyncio.CancelledError:
            print("[DroneController]: XBee veri alma döngüsü iptal edildi.")
        except Exception as e:
            print(f"[DroneController]: XBee veri alma döngüsünde hata: {e}")
        print("[DroneController]: XBee veri alma döngüsü durduruldu.")


    async def _process_xbee_package(self, package_data: dict) -> None:
        """
        Gelen XBee paketlerini tipine göre işler.
        """
        print("\n--- Gelen XBee Paketi İşleniyor... ---")
        if "error" in package_data:
            print(f"  Paket işleme hatası: {package_data['error']}")
            if "raw_data_hex" in package_data:
                print(f"  Ham Veri (Hex): {package_data['raw_data_hex']}")
            if "source_addr" in package_data:
                print(f"  Kaynak Adres: {package_data['source_addr']}")
            return

        package_type = package_data.get('t')
        sender_id = package_data.get('s')
        params = package_data.get('p', {})

        print(f"  Tip: {package_type}, Gönderen: {sender_id}, Parametreler: {params}")

        if package_type == "G":
            # Diğer dronların GPS verisi, APF için kullanılabilir
            if sender_id != self.DRONE_ID: # Kendi gönderdiğimiz paketi tekrar işlemeyelim
                latitude = params.get('x') / 1000000.0 if params.get('x') is not None else None
                longitude = params.get('y') / 1000000.0 if params.get('y') is not None else None
                if latitude is not None and longitude is not None:
                    self.apf_controller.update_other_drone_position(sender_id, latitude, longitude)
                    print(f"    Diğer drone ({sender_id}) GPS verisi alındı: Lat={latitude}, Lon={longitude}")
        elif package_type == "H":
            print(f"    El sıkışma paketi alındı: Gönderen={sender_id}")
            # El sıkışma yanıtı gönderme mantığı eklenebilir
        elif package_type == "W":
            waypoint_id = sender_id
            latitude = params.get('x') / 1000000.0 if params.get('x') is not None else None
            longitude = params.get('y') / 1000000.0 if params.get('y') is not None else None
            altitude = params.get('a', 0.0) # Varsayılan irtifa
            heading = params.get('h', 0) # Varsayılan yön
            if latitude is not None and longitude is not None:
                self.waypoint_manager.add(waypoint_id, latitude, longitude, altitude, heading)
                print(f"    Waypoint eklendi/güncellendi: ID={waypoint_id}, Lat={latitude}, Lon={longitude}, Alt={altitude}, Hed={heading}")
        elif package_type == "w":
            waypoint_id = sender_id
            self.waypoint_manager.remove(waypoint_id)
            print(f"    Waypoint silme isteği alındı: ID={waypoint_id}")
        elif package_type == "O":
            mission_id = sender_id
            drone_ids = params.get('d', [])
            formation = params.get('f', 'None')
            waypoint_ids = params.get('wp', [])
            print(f"    Görev emri alındı: Görev ID={mission_id}, Dronlar={drone_ids}, Formasyon={formation}, Waypointler={waypoint_ids}")
            # Kendi ID'miz görevden sorumlu dronlar arasında mı kontrol et
            if self.DRONE_ID in drone_ids:
                self.required_confirmations = len(drone_ids) # Beklenen onay sayısı
                await self.run_mission(mission_id, {"drone_ids": drone_ids, "formation": formation, "waypoint_ids": waypoint_ids})
            else:
                print(f"    Bu drone ({self.DRONE_ID}) görevden sorumlu değil.")
        elif package_type == "MC":
            mission_id_confirm = params.get('id', 'N/A')
            print(f"    Göreve başlama onayı alındı: Gönderen={sender_id}, Görev numarası={mission_id_confirm}")
            # Eğer aktif bir görev varsa ve onay bekliyorsak
            if self.current_mission and self.current_mission.mission_id == mission_id_confirm:
                self.mission_confirmations.add(sender_id)
                print(f"    Onaylar: {len(self.mission_confirmations)}/{self.required_confirmations}")
                if len(self.mission_confirmations) >= self.required_confirmations:
                    print(f"    Tüm onaylar alındı. Görev {mission_id_confirm} başlatılıyor!")
                    # Görevi başlatma sinyali gönderilebilir
                    if hasattr(self.current_mission, 'all_confirmed_event'):
                        self.current_mission.all_confirmed_event.set()
        elif package_type == "MS":
            status = params.get('status', 'unknown')
            print(f"    Görev durumu alındı: Gönderen={sender_id}, Durum={status}")
        else:
            print(f"    Bilinmeyen paket tipi alındı: {package_type}")


    async def goto_waypoint(self, waypoint_id: str, relative_alt: float = 10.0) -> bool:
        """
        Belirtilen waypoint ID'sine drone'u gönderir.
        PID ve APF çarpışma önleme mantığı burada entegre edilebilir.
        """
        waypoint = self.waypoint_manager.read(waypoint_id)
        if not waypoint:
            print(f"[DroneController]: Waypoint {waypoint_id} bulunamadı.")
            return False

        print(f"[DroneController]: Waypoint {waypoint_id} hedefine gidiliyor: Lat={waypoint.lat}, Lon={waypoint.lon}, Alt={waypoint.alt}")

        # Offboard modunu başlat
        await self.drone.offboard.set_position_ned(
            offboard.PositionNedYaw(0.0, 0.0, 0.0, 0.0))
        try:
            await self.drone.offboard.start()
        except offboard.OffboardError as error:
            print(f"[DroneController]: Offboard mod başlatılamadı: {error}")
            return False

        # Hedefe ulaşana kadar döngü
        # Bu kısım PID ve APF entegrasyonu için basitleştirilmiştir.
        # Gerçek bir uygulamada, sürekli konum güncellemeleri ve kontrol döngüsü gerekir.
        
        # Basit bir döngü ile hedefe gitme
        # PID ve APF burada konum komutlarını etkileyecek
        TOLERANCE_M = 0.5 # Hedefe yakınlık toleransı (metre)
        
        async for position_info in self.drone.telemetry.position():
            if self._stop_tasks_event.is_set():
                print("[DroneController]: Görev iptal edildi, waypoint'e gitme durduruldu.")
                break

            current_lat = position_info.latitude_deg
            current_lon = position_info.longitude_deg
            current_alt = position_info.absolute_altitude_m # Mutlak irtifa

            # Hedef irtifa (göreceli irtifa + kalkış irtifası)
            target_alt_abs = waypoint.alt # Waypoint'teki irtifa zaten mutlak olabilir, kontrol et.
                                          # Eğer waypoint.alt göreceli ise, home_altitude'u eklemek gerekir.
                                          # MAVSDK'da genellikle relative_alt kullanılır.
                                          # Şimdilik waypoint.alt'ı mutlak kabul edelim.

            # Hedef ve mevcut konum arası hata hesaplaması
            # Basit bir mesafe hesaplaması (gerçekte daha hassas coğrafi hesaplama gerekir)
            # Bu hatalar PID'ye beslenir
            error_lat = waypoint.lat - current_lat
            error_lon = waypoint.lon - current_lon
            error_alt = target_alt_abs - current_alt
            
            # PID çıktıları (hız veya ivme komutları olarak düşünülebilir)
            # dt değeri sabit bir zaman adımı olarak alınabilir (örn: 0.1 saniye)
            dt = 0.1 # Her döngüde geçen süre
            
            # PID ile hız komutları hesapla (basit bir örnek)
            # Bu değerler, offboard.set_velocity_ned() veya set_position_ned() ile kullanılabilir.
            # Burada offboard.set_position_ned() kullanıldığı için, PID çıktısını doğrudan konum farkına ekleyebiliriz.
            
            # PID'den gelen düzeltmeler
            correction_lat = self.pid_lat.calculate(error_lat, dt)
            correction_lon = self.pid_lon.calculate(error_lon, dt)
            correction_alt = self.pid_alt.calculate(error_alt, dt)
            
            # APF'den gelen çarpışma önleme vektörleri
            apf_lat_force, apf_lon_force = self.apf_controller.calculate_avoidance_vector(
                current_lat, current_lon, waypoint.lat, waypoint.lon
            )

            # Nihai hedef konum ayarlaması (PID ve APF ile)
            # Bu kısım, PID ve APF çıktılarının nasıl birleştirileceğine bağlıdır.
            # Genellikle PID hız veya ivme komutları üretirken, APF de bir kuvvet veya hız vektörü üretir.
            # Burada basitleştirilmiş bir konum düzeltmesi yapıyoruz.
            
            # Hedef konumda küçük ayarlamalar yaparak APF etkisini gösteriyoruz
            command_lat = waypoint.lat + correction_lat + apf_lat_force
            command_lon = waypoint.lon + correction_lon + apf_lon_force
            command_alt = waypoint.alt + correction_alt # İrtifa için APF şimdilik düşünülmüyor

            # MAVSDK offboard komutunu gönder
            # MAVSDK'da goto_location daha yüksek seviye bir fonksiyondur ve genellikle tercih edilir.
            # set_position_ned daha düşük seviye kontrol için kullanılır.
            # Burada goto_location kullanmak daha uygun olacaktır.
            await self.drone.action.goto_location(command_lat, command_lon, command_alt, waypoint.hed)


            # Hedefe ulaşıldı mı kontrolü (basit mesafe kontrolü)
            # Gerçek bir uygulamada, hedef konuma yeterince yaklaşıldığında döngüden çıkılır.
            # MAVSDK'nın kendi konum telemetrisini kullanarak mesafeyi hesaplayın.
            
            # Basit Öklid mesafesi (coğrafi koordinatlar için uygun değil, sadece örnek)
            # Gerçek bir mesafe hesaplaması için haversine veya Vincenty formülleri kullanılmalıdır.
            # Şimdilik basit bir yaklaşım:
            # Yaklaşık 1 derece enlem ~ 111 km
            # Yaklaşık 1 derece boylam ~ 111 * cos(latitude) km
            # Bu yüzden sadece farkların karekökünü almak yeterli değildir.
            # Ancak test amaçlı olarak, küçük mesafelerde bu yaklaşım bir fikir verebilir.
            
            # Örnek: Basit bir mesafe kontrolü (daha doğru bir coğrafi mesafe hesaplaması gereklidir)
            # Burada sadece lat/lon farkını kullanıyoruz, bu doğru bir mesafe değil.
            # Gerçek bir uygulamada `geopy` gibi kütüphanelerle mesafe hesaplanmalıdır.
            
            # Mesafe hesaplaması için basit bir placeholder:
            # distance_to_target = ((waypoint.lat - current_lat)**2 + (waypoint.lon - current_lon)**2)**0.5 * 111320 # Yaklaşık metreye çevirme
            # MavSDK'nın kendi `distance_to_waypoint` gibi bir metodu varsa o tercih edilmelidir.
            # Şimdilik, hedef konuma yeterince yakın olup olmadığını kontrol etmek için basit bir eşik kullanacağız.
            
            # MAVSDK'nın `distance_to_point` metodunu kullanabiliriz (telemetry'de yok, ancak kendimiz hesaplayabiliriz).
            # Veya `goto_location` zaten hedefe ulaştığında tamamlanır.
            # `goto_location` asenkron bir çağrı olduğu için, bu döngü `goto_location`'ın kendisi tamamlanana kadar beklemeyecektir.
            # Bu nedenle, `goto_location`'ı her döngüde çağırmak yerine, bir kez çağırıp tamamlanmasını beklemek daha mantıklı olabilir.
            # Ancak PID/APF entegrasyonu için sürekli komut göndermek gerekir.
            
            # Bu kısım, `goto_location`'ın nasıl kullanıldığına bağlı olarak değişebilir.
            # Eğer `goto_location`'ı sürekli olarak güncellenen PID/APF komutlarıyla kullanacaksak,
            # döngüde kalıp mesafeyi kontrol etmeye devam etmeliyiz.
            
            # Mevcut yaklaşımda, `goto_location` her döngüde çağrıldığından,
            # hedefe ulaşma kontrolünü manuel olarak yapmamız gerekiyor.
            
            # Basit bir mesafe kontrolü (doğruluk için Haversine formülü önerilir)
            # Bu sadece test amaçlı bir yaklaşımdır.
            distance_lat = abs(waypoint.lat - current_lat) * 111320 # Yaklaşık metre
            distance_lon = abs(waypoint.lon - current_lon) * 111320 * abs(math.cos(math.radians(current_lat))) # Yaklaşık metre
            horizontal_distance = (distance_lat**2 + distance_lon**2)**0.5
            
            if horizontal_distance < TOLERANCE_M and abs(target_alt_abs - current_alt) < TOLERANCE_M:
                print(f"[DroneController]: Waypoint {waypoint_id} hedefine ulaşıldı.")
                break

            await asyncio.sleep(dt) # Kontrol döngüsü hızı

        # Offboard modunu durdur
        try:
            await self.drone.offboard.stop()
        except offboard.OffboardError as error:
            print(f"[DroneController]: Offboard mod durdurulamadı: {error}")
            return False
        
        return True


    def load_missions(self, missions_folder: str = "missions"):
        """
        'missions' klasöründeki görev sınıflarını dinamik olarak yükler.
        """
        # Mevcut dosyanın dizinini al (controllers/drone_controller.py)
        current_dir = os.path.dirname(os.path.abspath(__file__))
        # Proje kök dizinine git (controllers'ın bir üstü)
        project_root = os.path.dirname(current_dir)
        # Görevler klasörünün tam yolu
        full_missions_path = os.path.join(project_root, missions_folder)

        if not os.path.exists(full_missions_path):
            print(f"[DroneController]: '{full_missions_path}' görev klasörü bulunamadı.")
            return

        print(f"[DroneController]: Görevler '{full_missions_path}' klasöründen yükleniyor...")
        
        # mission_controller.py'dan Mission base sınıfını import et
        mission_controller_path = os.path.join(current_dir, "mission_controller.py")
        if not os.path.exists(mission_controller_path):
            print(f"[DroneController]: 'mission_controller.py' bulunamadı. Görevler yüklenemiyor.")
            return
        
        spec = importlib.util.spec_from_file_location("mission_controller", mission_controller_path)
        if spec is None:
            print(f"[DroneController]: 'mission_controller.py' için spec oluşturulamadı.")
            return
        
        mission_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mission_module)
        
        # Mission base sınıfını al
        MissionBase = getattr(mission_module, 'Mission', None)
        if MissionBase is None:
            print("[DroneController]: 'mission_controller.py' içinde 'Mission' sınıfı bulunamadı.")
            return

        for filename in os.listdir(full_missions_path):
            if filename.endswith(".py") and filename != "__init__.py":
                module_name = filename[:-3] # .py uzantısını kaldır
                file_path = os.path.join(full_missions_path, filename)
                
                try:
                    spec = importlib.util.spec_from_file_location(module_name, file_path)
                    if spec is None:
                        print(f"  Uyarı: '{filename}' için spec oluşturulamadı, atlanıyor.")
                        continue
                    
                    module = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(module)

                    for name, obj in module.__dict__.items():
                        # Yüklenen her sınıfı kontrol et
                        if isinstance(obj, type) and issubclass(obj, MissionBase) and obj is not MissionBase:
                            # Sadece MissionBase'den türeyen ve kendisi olmayan sınıfları al
                            mission_id = name # Sınıf adını görev ID olarak kullan
                            self.missions[mission_id] = obj
                            print(f"  Görev yüklendi: {mission_id} (Dosya: {filename})")
                            break # Her dosyadan sadece bir görev sınıfı beklenir
                except Exception as e:
                    print(f"  Hata: '{filename}' dosyasından görev yüklenirken sorun oluştu: {e}")
        
        if not self.missions:
            print("[DroneController]: Hiç görev yüklenemedi.")
        else:
            print(f"[DroneController]: Toplam {len(self.missions)} görev yüklendi.")


    async def run_mission(self, mission_id: str, params: dict) -> bool:
        """
        Belirtilen görev ID'sine sahip görevi başlatır.
        """
        mission_class = self.missions.get(mission_id)
        if not mission_class:
            print(f"[DroneController]: Görev '{mission_id}' bulunamadı.")
            return False

        print(f"[DroneController]: Görev '{mission_id}' başlatılıyor...")
        self.current_mission = mission_class(self, params) # DroneController'ı ve parametreleri göreve geçir
        self.mission_confirmations.clear() # Yeni görev için onayları sıfırla
        
        try:
            await self.current_mission.start()
            print(f"[DroneController]: Görev '{mission_id}' tamamlandı.")
            # Görev tamamlandığında durum paketi gönderilebilir
            status_package = XBeePackage(
                package_type="MS",
                sender=self.DRONE_ID,
                params={"status": "successful", "mission_id": mission_id}
            )
            self.xbee_controller.send(status_package)
            return True
        except asyncio.CancelledError:
            print(f"[DroneController]: Görev '{mission_id}' iptal edildi.")
            status_package = XBeePackage(
                package_type="MS",
                sender=self.DRONE_ID,
                params={"status": "cancelled", "mission_id": mission_id}
            )
            self.xbee_controller.send(status_package)
            return False
        except Exception as e:
            print(f"[DroneController]: Görev '{mission_id}' çalıştırılırken hata oluştu: {e}")
            status_package = XBeePackage(
                package_type="MS",
                sender=self.DRONE_ID,
                params={"status": "failed", "mission_id": mission_id, "error": str(e)}
            )
            self.xbee_controller.send(status_package)
            return False
        finally:
            self.current_mission = None # Görev tamamlandığında veya başarısız olduğunda sıfırla


# --- Test için Ana Fonksiyon ---
async def main():
    # Kullanıcıdan XBee portunu ve drone bağlantı adresini al
    print('XBee bağlantısı için port girin (örn: 0, 1, ... veya "default" için boş bırakın)')
    input_xbee_port_str = input('/dev/ttyUSB? : ')

    if input_xbee_port_str.strip() == "":
        xbee_port = "/dev/ttyUSB0"
    else:
        try:
            port_number = int(input_xbee_port_str)
            xbee_port = f"/dev/ttyUSB{port_number}"
        except ValueError:
            print("Geçersiz port girişi. Varsayılan olarak /dev/ttyUSB0 kullanılacak.")
            xbee_port = "/dev/ttyUSB0"

    print('Drone bağlantısı için adres girin (örn: "udpin://0.0.0.0:14540" veya "default" için boş bırakın)')
    input_drone_sys_address = input('Drone System Address: ')
    if input_drone_sys_address.strip() == "":
        drone_sys_address = "udpin://0.0.0.0:14540"
    else:
        drone_sys_address = input_drone_sys_address

    drone_controller = DroneController(sys_address=drone_sys_address, xbee_port=xbee_port)

    try:
        await drone_controller.connect()

        # Eğer XBee bağlantısı başarısız olursa çık
        # Drone bağlantısı, drone_controller.connect() içinde zaten kontrol ediliyor.
        # Eğer bu noktaya gelindiyse, drone bağlantısının başarılı olduğu varsayılır.
        if drone_controller.xbee_controller and not drone_controller.xbee_controller.connected:
            print("XBee bağlantı sorunları nedeniyle uygulama sonlandırılıyor.")
            return

        print("\nDrone Controller başarıyla başlatıldı.")
        print("Komutlar:")
        print("  'arm'    : Drone'u arm et")
        print("  'takeoff': Drone'u havalandır (arm edilmiş olmalı)")
        print("  'land'   : Drone'u indir")
        print("  'goto <waypoint_id>': Belirtilen waypointe git")
        print("  'addwp <id> <lat> <lon> <alt> <hed>': Waypoint ekle/güncelle")
        print("  'rmwp <id>': Waypoint sil")
        print("  'sendgps': Manuel GPS paketi gönder (test)")
        print("  'run <mission_id>': Görev başlat")
        print("  'list_missions': Yüklü görevleri listele")
        print("  'q'      : Çıkış")

        # Kullanıcı komutlarını dinleyen döngü
        while True:
            command = await asyncio.to_thread(input, "Komut girin: ").strip().lower()

            if command == 'q':
                break
            elif command == 'arm':
                print("Drone arm ediliyor...")
                await drone_controller.drone.action.arm()
                print("Drone arm edildi.")
            elif command == 'takeoff':
                print("Drone havalanıyor...")
                await drone_controller.drone.action.takeoff()
                print("Drone havalandı.")
            elif command == 'land':
                print("Drone iniş yapıyor...")
                await drone_controller.drone.action.land()
                print("Drone iniş yaptı.")
            elif command.startswith('goto '):
                parts = command.split()
                if len(parts) == 2:
                    waypoint_id = parts[1]
                    await drone_controller.goto_waypoint(waypoint_id)
                else:
                    print("Kullanım: goto <waypoint_id>")
            elif command.startswith('addwp '):
                parts = command.split()
                if len(parts) == 6:
                    try:
                        wp_id = parts[1]
                        lat = float(parts[2])
                        lon = float(parts[3])
                        alt = float(parts[4])
                        hed = float(parts[5])
                        drone_controller.waypoint_manager.add(wp_id, lat, lon, alt, hed)
                        print(f"Waypoint {wp_id} eklendi.")
                    except ValueError:
                        print("Geçersiz sayı formatı. Kullanım: addwp <id> <lat> <lon> <alt> <hed>")
                else:
                    print("Kullanım: addwp <id> <lat> <lon> <alt> <hed>")
            elif command.startswith('rmwp '):
                parts = command.split()
                if len(parts) == 2:
                    wp_id = parts[1]
                    drone_controller.waypoint_manager.remove(wp_id)
                else:
                    print("Kullanım: rmwp <id>")
            elif command == 'sendgps':
                # Sadece test amaçlı manuel GPS paketi gönderme
                async for position in drone_controller.drone.telemetry.position():
                    lat = position.latitude_deg
                    lon = position.longitude_deg
                    gps_package = XBeePackage(
                        package_type="G",
                        sender=drone_controller.DRONE_ID,
                        params={
                            "x": int(lat * 1000000),
                            "y": int(lon * 1000000)
                        }
                    )
                    drone_controller.xbee_controller.send(gps_package)
                    print(f"Manuel GPS paketi gönderildi: Lat={lat}, Lon={lon}")
                    break # Sadece bir kez gönder
            elif command.startswith('run '):
                parts = command.split()
                if len(parts) >= 2:
                    mission_id = parts[1]
                    # Basit bir parametre ayrıştırma örneği
                    # Gerçek uygulamada daha gelişmiş bir parser gerekebilir
                    mission_params = {}
                    if len(parts) > 2:
                        # Örnek: run MyMission d=1,2,3 f=V wp=wp1,wp2
                        # Basit bir key-value ayrıştırma
                        for param_str in parts[2:]:
                            if '=' in param_str:
                                key, value = param_str.split('=', 1)
                                if ',' in value: # Virgülle ayrılmış listeler için
                                    mission_params[key] = value.split(',')
                                else:
                                    mission_params[key] = value
                            else:
                                print(f"Uyarı: Geçersiz parametre formatı '{param_str}'. 'key=value' şeklinde olmalı.")
                    
                    await drone_controller.run_mission(mission_id, mission_params)
                else:
                    print("Kullanım: run <mission_id> [param1=value1 param2=value2 ...]")
            elif command == 'list_missions':
                if drone_controller.missions:
                    print("Yüklü Görevler:")
                    for mid in drone_controller.missions.keys():
                        print(f"  - {mid}")
                else:
                    print("Hiç görev yüklenemedi.")
            else:
                print("Bilinmeyen komut.")

    except Exception as e:
        print(f"Uygulama hatası: {e}")
    finally:
        await drone_controller.disconnect()
        print("Uygulama sonlandırıldı.")

if __name__ == "__main__":
    # asyncio.run() Python 3.7+ gerektirir.
    # Windows'ta bazen asyncio ile ilgili sorunlar olabilir, platform spesifik ayarlamalar gerekebilir.
    asyncio.run(main())

