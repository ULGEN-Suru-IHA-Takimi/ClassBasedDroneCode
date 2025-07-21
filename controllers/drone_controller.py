#!/usr/bin/env python3

import asyncio
import threading
import time
import json
import os
import importlib.util
import math
import sys

# Projenin kök dizinini Python yoluna ekle
# Bu, 'connect' ve 'controllers' gibi kardeş dizinlerdeki modüllerin bulunmasını sağlar.
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mavsdk import System, telemetry, offboard, action
from simple_pid import PID # simple-pid kütüphanesi import edildi

# Kendi modüllerimizi import ediyoruz
from connect.drone_connection import DroneConnection
from controllers.xbee_controller import XBeeController, XBeePackage
from controllers.waypoint_controller import waypoints, Waypoint
# Mission temel sınıfını doğrudan import ediyoruz
from controllers.mission_controller import Mission as MissionBase


class APF: # Artificial Potential Field (Yapay Potansiyel Alan)
    """
    Yapay Potansiyel Alan (APF) tabanlı çarpışma önleme.
    Basit bir itme kuvveti modeli kullanır.
    """
    def __init__(self, drone_id: str):
        self.drone_id = drone_id
        # Çarpışma önleme için diğer dronların konumları burada tutulabilir
        self.other_drone_positions = {} # {drone_id: (lat, lon, alt)}
        self.repulsion_distance = 15.0 # Metre cinsinden itme kuvveti uygulama mesafesi
        self.repulsion_strength = 0.5 # İtme kuvvetinin şiddeti

    def update_other_drone_position(self, drone_id: str, lat: float, lon: float, alt: float):
        """
        Diğer dronların konumunu günceller.
        """
        if drone_id != self.drone_id: # Kendi konumumuzu takip etmiyoru
            self.other_drone_positions[drone_id] = (lat, lon, alt)
            # print(f"APF: Drone {drone_id} konumu güncellendi: Lat={lat}, Lon={lon}, Alt={alt}")

    def calculate_avoidance_vector(self, current_lat, current_lon, current_alt):
        """
        Mevcut konuma göre çarpışma önleme vektörünü hesaplar.
        Args:
            current_lat (float): Mevcut enlem.
            current_lon (float): Mevcut boylam.
            current_alt (float): Mevcut irtifa.
        Returns:
            tuple: (avoidance_north_force, avoidance_east_force, avoidance_down_force)
                   Hız komutlarına eklenecek metre/saniye cinsinden vektörler.
        """
        avoidance_north_force = 0.0
        avoidance_east_force = 0.0
        avoidance_down_force = 0.0
        
        # Yaklaşık 1 derece enlem ~ 111320 metre
        # Yaklaşık 1 derece boylam ~ 111320 * cos(latitude_radians) metre
        lat_to_m = 111320.0
        lon_to_m = 111320.0 * math.cos(math.radians(current_lat))

        for other_id, (other_lat, other_lon, other_alt) in self.other_drone_positions.items():
            # Mesafeyi metre cinsinden hesapla
            delta_lat_m = (current_lat - other_lat) * lat_to_m
            delta_lon_m = (current_lon - other_lon) * lon_to_m
            delta_alt_m = current_alt - other_alt # İrtifa farkı

            distance_2d = math.sqrt(delta_lat_m**2 + delta_lon_m**2)
            distance_3d = math.sqrt(distance_2d**2 + delta_alt_m**2)

            if distance_3d < self.repulsion_distance and distance_3d > 0.1: # Çok yakınsa ve sıfır değilse
                # İtme kuvveti hesapla (mesafe azaldıkça kuvvet artsın)
                force_magnitude = self.repulsion_strength * (self.repulsion_distance - distance_3d) / self.repulsion_distance
                
                # Yön vektörü (diğer drona doğru)
                direction_lat = delta_lat_m / distance_2d if distance_2d > 0 else 0
                direction_lon = delta_lon_m / distance_2d if distance_2d > 0 else 0
                direction_alt = delta_alt_m / distance_3d if distance_3d > 0 else 0

                avoidance_north_force += force_magnitude * direction_lat
                avoidance_east_force += force_magnitude * direction_lon
                avoidance_down_force += force_magnitude * direction_alt # İrtifa için de itme

        return avoidance_north_force, avoidance_east_force, avoidance_down_force


class DroneController(DroneConnection):
    """
    DroneConnection'dan türeyen ana drone kontrol sınıfı.
    XBee iletişimi, waypoint yönetimi ve görev yürütme yeteneklerini içerir.
    """
    DRONE_ID = "1" # Bu dronun benzersiz ID'si

    def __init__(self, sys_address: str = "udpin://0.0.0.0:14540", xbee_port: str = "/dev/ttyUSB0"):
        super().__init__(sys_address)
        print(f"[DroneController]: Initializing DroneController instance: {self}") # Debug output
        self.xbee_port = xbee_port
        self.xbee_controller: XBeeController = None
        self.waypoint_manager = waypoints() # Waypoint'leri yönetecek sınıf
        self.current_mission = None # Aktif görevi tutar
        self.missions = {} 

        self._gps_send_task = None
        self._xbee_receive_task = None
        self._monitor_armed_state_task = None # Reference for armed state monitoring task
        self._drone_armed_event = asyncio.Event() # Tracks whether the drone is armed
        self._stop_tasks_event = asyncio.Event() # To stop all async tasks

        self.apf_controller = APF(self.DRONE_ID) # APF for collision avoidance

        # Set for mission confirmations (for swarm UAVs)
        self.mission_confirmations = set() 
        self.required_confirmations = 0 # Number of confirmations expected for the mission

        # PID controllers (tuned values) - using simple-pid
        # Horizontal PID
        self.pid_north = PID(Kp=0.1, Ki=0.001, Kd=0.8, setpoint=0, sample_time=0.1, output_limits=(-5.0, 5.0)) 
        self.pid_east = PID(Kp=0.1, Ki=0.001, Kd=0.8, setpoint=0, sample_time=0.1, output_limits=(-5.0, 5.0))  
        # Vertical PID: Tuned for altitude holding
        # Kp, Ki, Kd values are negative because:
        #   - If current_alt < target_alt (error = target_alt - current_alt is positive),
        #     we want vel_down_pid to be negative (move UP). So Kp must be negative.
        #   - If current_alt > target_alt (error = target_alt - current_alt is negative),
        #     we want vel_down_pid to be positive (move DOWN). So Kp must be negative.
        # setpoint will be dynamically updated to target_alt
        self.pid_down = PID(Kp=-1.0, Ki=-0.01, Kd=-1.0, setpoint=0, sample_time=0.1, output_limits=(-5.0, 5.0))  # Tuned

        # Max horizontal speed of the drone is maintained
        self.drone_speed = 5.0 # Target speed of the drone (m/s)


    async def connect(self) -> None:
        """
        Connects to the drone and starts XBee communication.
        """
        await super().connect() # Call DroneConnection's connect method

        # Start XBee Controller
        print(f"[DroneController]: XBee device is being initialized via {self.xbee_port}...")
        self.xbee_controller = XBeeController(self.xbee_port)
        if not self.xbee_controller.connected:
            print("[DroneController]: Could not connect to XBee device. XBee communication disabled.")
            # If XBee connection fails, do not start other XBee dependent tasks
            return
        
        print("[DroneController]: XBee connection successful. Data listening is starting.")
        # Start the asynchronous task that listens for data from XBee
        self._xbee_receive_task = asyncio.create_task(self._xbee_receive_loop())

        # Start the task that monitors the drone's armed status
        self._monitor_armed_state_task = asyncio.create_task(self._monitor_armed_state()) # Task reference is held

        # Load missions
        print(f"[DroneController]: Attempting to load missions. Type of self: {type(self)}")
        print(f"[DroneController]: Attributes of self before load_missions: {dir(self)}") # Debug output
        self.load_missions() 
        
        print("[DroneController]: Drone Controller ready.")


    async def disconnect(self) -> None:
        """
        Closes drone and XBee connections, stops all asynchronous tasks.
        """
        print("[DroneController]: Drone Controller is shutting down...")
        self._stop_tasks_event.set() # Send stop signal to all async tasks

        # Wait for all async tasks to complete
        if self._gps_send_task and not self._gps_send_task.done():
            self._gps_send_task.cancel()
            try: await self._gps_send_task
            except asyncio.CancelledError: pass

        if self._xbee_receive_task and not self._xbee_receive_task.done():
            self._xbee_receive_task.cancel()
            try: await self._xbee_receive_task
            except asyncio.CancelledError: pass
        
        # Also cancel and wait for _monitor_armed_state_task
        if self._monitor_armed_state_task and not self._monitor_armed_state_task.done():
            self._monitor_armed_state_task.cancel()
            try: await self._monitor_armed_state_task
            except asyncio.CancelledError: pass

        # Close XBee Controller
        if self.xbee_controller:
            self.xbee_controller.disconnect()
        
        print("[DroneController]: Drone Controller shut down.")


    async def _monitor_armed_state(self) -> None:
        """Monitors the drone's armed status and starts/stops the GPS sending task."""
        print("[DroneController]: Drone armed status is being monitored...")
        try:
            async for is_armed in self.drone.telemetry.armed():
                if self._stop_tasks_event.is_set():
                    break # Exit loop if shutdown signal received

                if is_armed and not self._drone_armed_event.is_set():
                    print("[DroneController]: Drone armed. Starting GPS data transmission.")
                    self._drone_armed_event.set()
                    if not self._gps_send_task or self._gps_send_task.done():
                        self._gps_send_task = asyncio.create_task(self._send_gps_data_loop())
                elif not is_armed and self._drone_armed_event.is_set():
                    print("[DroneController]: Drone disarmed. Stopping GPS data transmission.")
                    self._drone_armed_event.clear()
                    if self._gps_send_task and not self._gps_send_task.done():
                        self._gps_send_task.cancel()
                        try: await self._gps_send_task
                        except asyncio.CancelledError: pass
        except asyncio.CancelledError:
            print("[DroneController]: Armed status monitoring task cancelled.")
        except Exception as e:
            print(f"[DroneController]: Error in armed status monitoring task: {e}")
        print("[DroneController]: Armed status monitoring task stopped.")


    async def _send_gps_data_loop(self) -> None:
        """
        Sends GPS data over XBee when the drone is armed.
        """
        print("[DroneController]: GPS data transmission loop started.")
        try:
            async for position in self.drone.telemetry.position():
                if self._stop_tasks_event.is_set() or not self._drone_armed_event.is_set():
                    break # Exit loop if shutdown or disarm signal received

                lat = position.latitude_deg
                lon = position.longitude_deg
                alt = position.absolute_altitude_m # Altitude information should also be sent for APF
                
                # Format for XBeePackage (int * 1000000)
                gps_package = XBeePackage(
                    package_type="G",
                    sender=self.DRONE_ID,
                    params={
                        "x": int(lat * 1000000),
                        "y": int(lon * 1000000),
                        "a": int(alt * 100) # Send altitude in centimeters
                    }
                )
                self.xbee_controller.send(gps_package)
                await asyncio.sleep(1) # Send every second
        except asyncio.CancelledError:
            print("[DroneController]: GPS data transmission loop cancelled.")
        except Exception as e:
            print(f"[DroneController]: Error in GPS data transmission loop: {e}")
        print("[DroneController]: GPS data transmission loop stopped.")


    async def _xbee_receive_loop(self) -> None:
        """
        Continuously listens for and processes incoming XBee packages.
        """
        print("[DroneController]: XBee data reception loop started.")
        try:
            while not self._stop_tasks_event.is_set() and self.xbee_controller.connected:
                incoming_package_json = self.xbee_controller.receive()
                if incoming_package_json:
                    await self._process_xbee_package(incoming_package_json)
                else:
                    await asyncio.sleep(0.01) # Short wait to avoid wasting CPU
        except asyncio.CancelledError:
            print("[DroneController]: XBee data reception loop cancelled.")
        except Exception as e:
            print(f"[DroneController]: Error in XBee data reception loop: {e}")
        print("[DroneController]: XBee data reception loop stopped.")


    async def _process_xbee_package(self, package_data: dict) -> None:
        """
        Processes incoming XBee packages based on their type.
        """
        # print("\n--- Incoming XBee Package Processing... ---") # Disabled for too much output
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

        # print(f"  Tip: {package_type}, Gönderen: {sender_id}, Parametreler: {params}") # Disabled for too much output

        if package_type == "G":
            # Diğer dronların GPS verisi, APF için kullanılabilir
            if sender_id != self.DRONE_ID: # Don't process our own sent package again
                latitude = params.get('x') / 1000000.0 if params.get('x') is not None else None
                longitude = params.get('y') / 1000000.0 if params.get('y') is not None else None
                altitude = params.get('a') / 100.0 if params.get('a') is not None else None # Convert from centimeters to meters
                if latitude is not None and longitude is not None and altitude is not None:
                    self.apf_controller.update_other_drone_position(sender_id, latitude, longitude, altitude)
                    # print(f"    Other drone ({sender_id}) GPS data received: Lat={latitude}, Lon={longitude}, Alt={altitude}")
        elif package_type == "H":
            print(f"    El sıkışma paketi alındı: Gönderen={sender_id}")
            # Handshake response sending logic can be added
        elif package_type == "W":
            waypoint_id = sender_id
            latitude = params.get('x') / 1000000.0 if params.get('x') is not None else None
            longitude = params.get('y') / 1000000.0 if params.get('y') is not None else None
            altitude = params.get('a', 0.0) # Default altitude
            heading = params.get('h', 0) # Default heading
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
            print(f"    Görev emri alındı: {params}")
            # Check if our ID is among the drones responsible for the mission
            if self.DRONE_ID in drone_ids:
                mission_info_for_run = self.missions.get(mission_id)
                if mission_info_for_run:
                    mission_name_for_run = mission_info_for_run['name']
                    await self.run_mission(mission_id, params)
                else:
                    print(f"    Hata: Görev ID {mission_id} için isim bilgisi bulunamadı. Görev çalıştırılamıyor.")
            else:
                print(f"    Bu drone ({self.DRONE_ID}) görevden sorumlu değil.")
        elif package_type == "MC":
            mission_id_confirm = params.get('id', 'N/A')
            print(f"    Göreve başlama onayı alındı: Gönderen={sender_id}, Görev numarası={mission_id_confirm}")
            # If there is an active mission and we are waiting for confirmation
            if self.current_mission and self.current_mission.mission_id == mission_id_confirm:
                self.mission_confirmations.add(sender_id)
                print(f"    Onaylar: {len(self.mission_confirmations)}/{self.required_confirmations}")
                if len(self.mission_confirmations) >= self.required_confirmations: 
                    self.current_mission.all_confirmed_event.set()
        elif package_type == "MS":
            status = params.get('status', 'unknown')
            print(f"    Görev durumu alındı: Gönderen={sender_id}, Durum={status}")
        else:
            print(f"    Bilinmeyen paket tipi alındı: {package_type}")

    async def post_takeoff_altitude_check(self, target_altitude: float, tolerance: float, timeout: float):
        """
        Havalandıktan sonra drone'un belirli bir irtifaya ulaşıp ulaşmadığını kontrol eder.
        Belirtilen süre içinde ulaşamazsa iniş yapar.
        """
        print(f"[DroneController]: Havalanma sonrası irtifa kontrolü başlatıldı. Hedef: {target_altitude}m, Tolerans: {tolerance}m, Zaman Aşımı: {timeout}s")
        start_time = time.time()
        
        position_async_iterator = self.drone.telemetry.position().__aiter__()

        while time.time() - start_time < timeout:
            try:
                position_info = await position_async_iterator.__anext__()
                current_alt = position_info.absolute_altitude_m
                
                print(f"  [DEBUG-Havalanma Kontrolü] Mevcut İrtifa: {current_alt:.2f}m (Hedef: {target_altitude}m)")

                if abs(current_alt - target_altitude) <= tolerance:
                    print(f"[DroneController]: Havalanma sonrası güvenli irtifaya ulaşıldı ({current_alt:.2f}m).")
                    return True
            except StopAsyncIteration:
                print("[DroneController]: Telemetry akışı sona erdi (havalanma kontrolü).")
                break
            except asyncio.CancelledError:
                print("[DroneController]: Havalanma sonrası irtifa kontrolü iptal edildi.")
                break
            except Exception as e:
                print(f"[DroneController]: Havalanma sonrası irtifa kontrolünde hata: {e}")
                break
            await asyncio.sleep(0.1) # Daha sık kontrol et

        print(f"[DroneController]: Uyarı: Havalanma sonrası güvenli irtifaya {timeout} saniye içinde ulaşılamadı. Otomatik iniş başlatılıyor.")
        await self.drone.action.land()
        print("[DroneController]: Otomatik iniş tamamlandı.")
        return False


    async def goto_location(self, target_lat: float, target_lon: float, target_alt: float, target_hed: float) -> bool:
        """
        Belirtilen koordinatlara PID ve APF tabanlı hız kontrolü ile drone'u gönderir.
        Bu fonksiyon, goto_waypoint tarafından dahili olarak kullanılır.
        Önce hedef irtifaya ulaşır, sonra yatayda hareket ederken irtifayı korur.
        """
        print(f"[DroneController]: Hedef koordinatlara gidiliyor: Lat={target_lat:.6f}, Lon={target_lon:.6f}, Alt={target_alt}m, Hed={target_hed} derece")

        # Offboard modunu başlat
        # Başlangıçta drone'un mevcut konumunda kalması için sıfır hız komutu gönder
        await self.drone.offboard.set_velocity_ned(
            offboard.VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        try:
            await self.drone.offboard.start()
            print("[DroneController]: Offboard modu başlatıldı.")
        except offboard.OffboardError as error:
            print(f"[DroneController]: Offboard mod başlatılamadı: {error}")
            return False

        TOLERANCE_M = 1.0 # Yatay hedefe yakınlık toleransı (metre)
        ALT_TOLERANCE_M = 0.5 # İrtifa toleransı (metre)
        SPEED_TOLERANCE = 0.5 # Hız toleransı (m/s)
        
        # Yaklaşık 1 derece enlem ~ 111320 metre
        lat_to_m = 111320.0
        
        # Reset PID controllers
        self.pid_north.reset()
        self.pid_east.reset()
        self.pid_down.reset()

        # KRİTİK DÜZELTME: Dikey PID'nin setpoint'ini hedef irtifaya ayarla
        self.pid_down.setpoint = target_alt

        try:
            position_async_iterator = self.drone.telemetry.position().__aiter__()
            velocity_async_iterator = self.drone.telemetry.velocity_ned().__aiter__()

            # --- FAZ 1: Hedef irtifaya ulaş ---
            print("[DroneController]: Hedef irtifaya çıkılıyor/iniliyor...")
            while not self._stop_tasks_event.is_set():
                try:
                    position_info = await position_async_iterator.__anext__()
                    velocity_info = await velocity_async_iterator.__anext__()
                except StopAsyncIteration:
                    print("[DroneController]: Telemetry akışı sona erdi (irtifa fazı).")
                    break
                except asyncio.CancelledError:
                    raise

                current_alt = position_info.absolute_altitude_m
                current_vel_down = velocity_info.down_m_s

                # Hata D sadece loglama için kullanılıyor
                error_down_m_for_log = (target_alt - current_alt) 

                print(f"  [DEBUG-İrtifa Fazı] Mevcut Alt: {current_alt:.2f}m, Hedef Alt: {target_alt:.2f}m, Hata D: {error_down_m_for_log:.2f}m, Mevcut Hız D: {current_vel_down:.2f}m/s")
                
                # PID bileşenlerini al ve yazdır
                p_term, i_term, d_term = self.pid_down.components
                print(f"  [DEBUG-İrtifa Fazı] PID Bileşenleri (P, I, D): ({p_term:.2f}, {i_term:.2f}, {d_term:.2f})")

                # Check if target altitude is reached and stable
                if abs(error_down_m_for_log) < ALT_TOLERANCE_M and abs(current_vel_down) < SPEED_TOLERANCE:
                    print("[DroneController]: Hedef irtifaya ulaşıldı ve stabil.")
                    break

                # KRİTİK DÜZELTME: Dikey hız komutunu PID'ye current_alt'ı vererek hesapla
                vel_down_pid = self.pid_down(current_alt) 

                # Dikey hızı sınırla (simple-pid output_limits ile de sınırlanır, bu ek bir güvenlik)
                max_vertical_vel = 5.0 # m/s
                if abs(vel_down_pid) > max_vertical_vel:
                    vel_down_pid = math.copysign(max_vertical_vel, vel_down_pid) 

                print(f"  [DEBUG-İrtifa Fazı] PID Çıkışı (vel_down_pid): {vel_down_pid:.2f}m/s") # Added debug for PID output

                # Sadece dikey hız komutu gönder, yatayda sabit kal
                # KRİTİK DÜZELTME: vel_down_pid'in önündeki eksi işareti kaldırıldı
                await self.drone.offboard.set_velocity_ned(
                    offboard.VelocityNedYaw(0.0, 0.0, vel_down_pid, target_hed)
                )
                await asyncio.sleep(self.pid_down.sample_time) # Use PID's sample time

            # --- FAZ 2: Hedef irtifada yatayda hareket et ---
            print("[DroneController]: Yatay harekete başlanıyor, irtifa korunacak.")
            # Reset horizontal PIDs (they were not used in altitude phase)
            self.pid_north.reset()
            self.pid_east.reset()
            # pid_down'ın setpoint'i zaten ayarlı ve kullanılmaya devam edecek

            while not self._stop_tasks_event.is_set():
                try:
                    position_info = await position_async_iterator.__anext__()
                    velocity_info = await velocity_async_iterator.__anext__()
                except StopAsyncIteration:
                    print("[DroneController]: Telemetry akışı sona erdi (yatay fazı).")
                    break
                except asyncio.CancelledError:
                    raise

                current_lat = position_info.latitude_deg
                current_lon = position_info.longitude_deg
                current_alt = position_info.absolute_altitude_m

                current_vel_north = velocity_info.north_m_s
                current_vel_east = velocity_info.east_m_s
                current_vel_down = velocity_info.down_m_s

                lon_to_m = 111320.0 * math.cos(math.radians(current_lat))

                # Yatay hatalar
                error_north_m = (target_lat - current_lat) * lat_to_m
                error_east_m = (target_lon - current_lon) * lon_to_m
                
                # Dikey hata (irtifayı korumak için) - sadece loglama için
                error_down_m_for_log = (target_alt - current_alt)

                horizontal_distance = math.sqrt(error_north_m**2 + error_east_m**2)
                
                print(f"  [DEBUG-Yatay Fazı] Mevcut: Lat={current_lat:.6f}, Lon={current_lon:.6f}, Alt={current_alt:.2f}m, Hız D: {current_vel_down:.2f}m/s") 
                print(f"  [DEBUG-Yatay Fazı] Hedef: Lat={target_lat:.6f}, Lon={target_lon:.6f}, Alt={target_alt:.2f}m")
                print(f"  [DEBUG-Yatay Fazı] Hata (m): N={error_north_m:.2f}, E={error_east_m:.2f}, D={error_down_m_for_log:.2f}")
                print(f"  [DEBUG-Yatay Fazı] Yatay Mesafe: {horizontal_distance:.2f}m")
                print(f"  [DEBUG-Yatay Fazı] Mevcut Hız (m/s): N={current_vel_north:.2f}, E={current_vel_east:.2f}")

                # Has target been reached? (Horizontal and Vertical tolerances)
                if horizontal_distance < TOLERANCE_M and abs(error_down_m_for_log) < ALT_TOLERANCE_M:
                    # Try to keep the drone fixed at the target point
                    print(f"[DroneController]: Hedefe {horizontal_distance:.2f}m yatayda ve {abs(error_down_m_for_log):.2f}m dikeyde ulaşıldı. Sabitleniyor...")
                    for _ in range(10): # Wait for 1 second (0.1s * 10) to stabilize
                        await self.drone.offboard.set_velocity_ned(
                            offboard.VelocityNedYaw(0.0, 0.0, 0.0, target_hed)
                        )
                        await asyncio.sleep(0.1)
                    print("[DroneController]: Drone sabitlendi. Görev tamamlandı.")
                    break # Exit loop

                # Calculate PID outputs (velocity commands)
                vel_north_pid = self.pid_north(error_north_m)
                vel_east_pid = self.pid_east(error_east_m)
                # Use PID for vertical velocity control
                vel_down_pid = self.pid_down(current_alt) # KRİTİK DÜZELTME: Input is current_alt

                # PID bileşenlerini al ve yazdır (yatay faz için de)
                p_term_n, i_term_n, d_term_n = self.pid_north.components
                p_term_e, i_term_e, d_term_e = self.pid_east.components
                p_term_d, i_term_d, d_term_d = self.pid_down.components
                print(f"  [DEBUG-Yatay Fazı] PID Bileşenleri (N-P, I, D): ({p_term_n:.2f}, {i_term_n:.2f}, {d_term_n:.2f})")
                print(f"  [DEBUG-Yatay Fazı] PID Bileşenleri (E-P, I, D): ({p_term_e:.2f}, {i_term_e:.2f}, {d_term_e:.2f})")
                print(f"  [DEBUG-Yatay Fazı] PID Bileşenleri (D-P, I, D): ({p_term_d:.2f}, {i_term_d:.2f}, {d_term_d:.2f})")


                # Calculate avoidance vectors from APF
                avoid_north, avoid_east, avoid_down = self.apf_controller.calculate_avoidance_vector(
                    current_lat, current_lon, current_alt
                )

                print(f"  [DEBUG-Yatay Fazı] PID Çıkışları: N={vel_north_pid:.2f}, E={vel_east_pid:.2f}, D={vel_down_pid:.2f}")
                print(f"  [DEBUG-Yatay Fazı] APF Kaçınma: N={avoid_north:.2f}, E={avoid_east:.2f}, D={avoid_down:.2f}")

                # Calculate final velocity commands: PID outputs + APF avoidance
                command_vel_north = vel_north_pid + avoid_north
                command_vel_east = vel_east_pid + avoid_east
                command_vel_down = vel_down_pid + avoid_down # Command from vertical PID + APF

                # Print raw D command (before limiting)
                print(f"  [DEBUG-Yatay Fazı] Ham D Komutu (sınırlamadan önce): {command_vel_down:.2f}")

                # Limit velocity commands
                current_horizontal_command_speed = math.sqrt(command_vel_north**2 + command_vel_east**2)
                if current_horizontal_command_speed > self.drone_speed:
                    scale_factor = self.drone_speed / current_horizontal_command_speed
                    command_vel_north *= scale_factor
                    command_vel_east *= scale_factor
                
                max_vertical_vel = 5.0 # m/s
                if abs(command_vel_down) > max_vertical_vel:
                    command_vel_down = math.copysign(max_vertical_vel, command_vel_down) 

                overall_max_vel = 5.0 # m/s
                current_overall_command_speed = math.sqrt(command_vel_north**2 + command_vel_east**2 + command_vel_down**2)
                if current_overall_command_speed > overall_max_vel:
                    scale_factor = overall_max_vel / current_overall_command_speed
                    command_vel_north *= scale_factor
                    command_vel_east *= scale_factor
                    command_vel_down *= scale_factor
                
                print(f"  [DEBUG-Yatay Fazı] Nihai Hız Komutu: N={command_vel_north:.2f}, E={command_vel_east:.2f}, D={command_vel_down:.2f}, Yaw={target_hed:.2f}")

                # KRİTİK DÜZELTME: command_vel_down'ın önündeki eksi işareti kaldırıldı
                await self.drone.offboard.set_velocity_ned(
                    offboard.VelocityNedYaw(command_vel_north, command_vel_east, command_vel_down, target_hed)
                )
                
                await asyncio.sleep(self.pid_north.sample_time) # Use horizontal PID's sample time

        except asyncio.CancelledError:
            print("[DroneController]: Hedef koordinatlara gitme görevi iptal edildi.")
        except Exception as e:
            print(f"[DroneController]: Hedef koordinatlara gitme görevinde hata: {e}")
        finally:
            # Offboard modunu durdur (görev tamamlandığında veya hata oluştuğunda)
            try:
                await self.drone.offboard.stop()
                print("[DroneController]: Offboard modu durduruldu.")
            except offboard.OffboardError as error:
                print(f"[DroneController]: Offboard mod durdurulamadı: {error}")
            self.current_mission = None # Görev tamamlandığında veya başarısız olduğunda sıfırla
            return True # Görev tamamlandı veya iptal edildi


    async def goto_waypoint(self, waypoint_id: str) -> bool:
        """
        Belirtilen waypoint ID'sine drone'u gönderir.
        Bu fonksiyon, waypoint bilgilerini alıp goto_location'ı çağırır.
        """
        waypoint = self.waypoint_manager.read(waypoint_id)
        if not waypoint:
            print(f"[DroneController]: Waypoint {waypoint_id} bulunamadı.")
            return False

        # Waypoint'ten alınan koordinat ve yön bilgileri ile goto_location'ı çağır
        return await self.goto_location(waypoint.lat, waypoint.lon, waypoint.alt, waypoint.hed)


    def load_missions(self, missions_folder: str = "missions"):
        """
        'missions' klasöründeki görev sınıflarını dinamik olarak yükler.
        Görev ID'si olarak dosya adını kullanır (örn: '1.py' -> '1').
        """
        print("[DroneController]: Entering load_missions method.") 
        current_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(current_dir)
        full_missions_path = os.path.join(project_root, missions_folder)

        print(f"[DroneController]: Görev klasörü yolu: {full_missions_path}")

        if not os.path.exists(full_missions_path):
            print(f"[DroneController]: Hata! '{full_missions_path}' görev klasörü bulunamadı.")
            return

        print(f"[DroneController]: Görevler '{full_missions_path}' klasöründen yükleniyor...")
        
        for filename in os.listdir(full_missions_path):
            if filename.endswith(".py") and filename != "__init__.py":
                # Dosya adını görev ID'si olarak kullanıyoruz
                file_mission_id = filename[:-3] 
                file_path = os.path.join(full_missions_path, filename)
                
                print(f"  [DroneController]: '{filename}' dosyası işleniyor (Dosya ID: {file_mission_id})...")

                try:
                    spec = importlib.util.spec_from_file_location(f"mission_{file_mission_id}", file_path)
                    if spec is None:
                        print(f"  Uyarı: '{filename}' için spec oluşturulamadı, atlanıyor.")
                        continue
                    
                    module = importlib.util.module_from_spec(spec)
                    spec.loader.exec_module(module)

                    found_mission_class = None
                    for name, obj in module.__dict__.items():
                        # Sadece MissionBase'den türeyen ve kendisi olmayan sınıfları al
                        if isinstance(obj, type) and issubclass(obj, MissionBase) and obj is not MissionBase:
                            found_mission_class = obj
                            break # İlk bulunan geçerli görev sınıfını al

                    if found_mission_class:
                        # Sınıfın içindeki MISSION_ID ve mission_name özelliklerini alıyoruz
                        class_mission_id = getattr(found_mission_class, 'MISSION_ID', None)
                        mission_name = getattr(found_mission_class, 'mission_name', "Bilinmeyen Görev")

                        # Dosya adı ID'si ile sınıfın içindeki ID'yi karşılaştırabiliriz (isteğe bağlı tutarlılık kontrolü)
                        if class_mission_id and class_mission_id == file_mission_id:
                            self.missions[file_mission_id] = {
                                'class': found_mission_class, 
                                'name': mission_name,
                                'class_id': class_mission_id # Sınıfın kendi içindeki ID'yi de saklayabiliriz
                            }
                            print(f"  [DroneController]: Görev yüklendi: ID='{file_mission_id}', İsim='{mission_name}' (Dosya: {filename}, Sınıf: {found_mission_class.__name__})")
                        else:
                            print(f"  Uyarı: '{filename}' dosyasındaki '{found_mission_class.__name__}' sınıfının MISSION_ID ('{class_mission_id}') dosya adıyla ('{file_mission_id}') eşleşmiyor veya tanımlı değil. Bu görev yüklenmedi.")
                    else:
                        print(f"  Uyarı: '{filename}' dosyasında MissionBase'den türeyen geçerli bir görev sınıfı bulunamadı.")

                except Exception as e:
                    print(f"  Hata: '{filename}' dosyasından görev yüklenirken sorun oluştu: {e}")
        
        if not self.missions:
            print("[DroneController]: Hiç görev yüklenemedi.")
        else:
            print(f"[DroneController]: Toplam {len(self.missions)} görev yüklendi.")


    async def run_mission(self, mission_id: str, params: dict) -> bool:
        """
        Belirtilen görev ID'sine sahip görevi başlatır.
        mission_id burada dosya adından gelen ID'dir (örn: "1").
        """
        mission_info = self.missions.get(mission_id)
        if not mission_info:
            print(f"[DroneController]: Görev '{mission_id}' bulunamadı.")
            return False

        mission_class = mission_info['class']
        mission_name = mission_info['name']

        print(f"[DroneController]: Görev '{mission_id}' ({mission_name}) başlatılıyor...")
        
        self.current_mission = mission_class(self, mission_id, mission_name, params) 
        self.mission_confirmations.clear() # Yeni görev için onayları sıfırla
        
        try:
            await self.current_mission.start()
            print(f"[DroneController]: Görev '{mission_id}' ({mission_name}) tamamlandı.")
            # Görev tamamlandığında durum paketi gönderilebilir
            status_package = XBeePackage(
                package_type="MS",
                sender=self.DRONE_ID,
                params={"status": "successful", "mission_id": mission_id}
            )
            self.xbee_controller.send(status_package)
            return True
        except asyncio.CancelledError:
            print(f"[DroneController]: Görev '{mission_id}' ({mission_name}) iptal edildi.")
            status_package = XBeePackage(
                package_type="MS",
                sender=self.DRONE_ID,
                params={"status": "cancelled", "mission_id": mission_id}
            )
            self.xbee_controller.send(status_package)
            return False
        except Exception as e:
            print(f"[DroneController]: Görev '{mission_id}' ({mission_name}) çalıştırılırken hata oluştu: {e}")
            status_package = XBeePackage(
                package_type="MS",
                sender=self.DRONE_ID,
                params={"status": "failed", "mission_id": mission_id, "error": str(e)}
            )
            self.xbee_controller.send(status_package)
            return False
        finally:
            # Offboard modunu durdur (görev tamamlandığında veya hata oluştuğunda)
            try:
                await self.drone.offboard.stop()
                print("[DroneController]: Offboard modu durduruldu.")
            except offboard.OffboardError as error:
                print(f"[DroneController]: Offboard mod durdurulamadı: {error}")
            self.current_mission = None # Görev tamamlandığında veya başarısız olduğunda sıfırla
            return True # Görev tamamlandı veya iptal edildi


# --- Test için Ana Fonksiyon ---
async def main():
    # Kullanıcıdan XBee portunu ve drone bağlantı adresini al
    print('XBee bağlantısı için port girin (örn: 0, 1, ... veya "default" için boş bırakın)')
    input_xbee_port_str = await asyncio.to_thread(input, '/dev/ttyUSB? : ')

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
    input_drone_sys_address = await asyncio.to_thread(input, 'Drone System Address: ')
    if input_drone_sys_address.strip() == "":
        drone_sys_address = "udpin://0.0.0.0:14540"
    else:
        drone_sys_address = input_drone_sys_address

    drone_controller = DroneController(sys_address=drone_sys_address, xbee_port=xbee_port)

    try:
        await drone_controller.connect()

        if drone_controller.xbee_controller and not drone_controller.xbee_controller.connected:
            print("XBee bağlantı sorunları nedeniyle uygulama sonlandırılıyor.")
            return

        print("\nDrone Controller başarıyla başlatıldı.")
        def print_help():
            print("\nKomutlar:")
            print("  'arm'    : Drone'u arm et")
            print("  'takeoff': Drone'u havalandır (arm edilmiş olmalı)")
            print("  'land'   : Drone'u indir")
            print("  'goto <waypoint_id>': Belirtilen waypointe git (PID/APF kontrollü)")
            print("  'goto_coord <lat> <lon> <alt> <hed>': Belirtilen koordinatlara git (PID/APF kontrollü)")
            print("  'addwp <id> <lat> <lon> <alt> <hed>': Waypoint ekle/güncelle")
            print("  'rmwp <id>': Waypoint sil")
            print("  'sendgps': Manuel GPS paketi gönder (test)")
            print("  'run <mission_id> [param1=value1 param2=value2 ...]': Görev başlat (ID ve İsim ile listelenir)")
            print("  'list_missions': Yüklü görevleri ID ve İsimleri ile listeler")
            print("  'help'   : Bu komut listesini gösterir")
            print("  'q'      : Çıkış")
        
        print_help()

        while True:
            command = (await asyncio.to_thread(input, "Komut girin: ")).strip().lower()

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
                
                # Takeoff sonrası ek bekleme
                await asyncio.sleep(10) # Drone'a havalanması için 10 saniye süre ver

                # Anlık irtifa kontrolü
                try:
                    current_pos = await drone_controller.drone.telemetry.position().__anext__()
                    print(f"[DroneController]: Havalanma sonrası anlık irtifa: {current_pos.absolute_altitude_m:.2f}m")
                except Exception as e:
                    print(f"[DroneController]: Anlık irtifa alınamadı: {e}")

                # Added post-takeoff altitude check
                # Assuming default takeoff altitude is around 2.5m for initial stability
                await drone_controller.post_takeoff_altitude_check(target_altitude=2.5, tolerance=0.5, timeout=10) 
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
            elif command.startswith('goto_coord '): # Yeni komut eklendi
                parts = command.split()
                if len(parts) == 5:
                    try:
                        lat = float(parts[1])
                        lon = float(parts[2])
                        alt = float(parts[3])
                        hed = float(parts[4])
                        await drone_controller.goto_location(lat, lon, alt, hed)
                    except ValueError:
                        print("Geçersiz sayı formatı. Kullanım: goto_coord <lat> <lon> <alt> <hed>")
                else:
                    print("Kullanım: goto_coord <lat> <lon> <alt> <hed>")
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
                async for position in drone_controller.drone.telemetry.position():
                    lat = position.latitude_deg
                    lon = position.longitude_deg
                    alt = position.absolute_altitude_m
                    gps_package = XBeePackage(
                        package_type="G",
                        sender=drone_controller.DRONE_ID,
                        params={
                            "x": int(lat * 1000000),
                            "y": int(lon * 1000000),
                            "a": int(alt * 100) # Santimetre cinsinden gönder
                        }
                    )
                    drone_controller.xbee_controller.send(gps_package) 
                    print(f"Manuel GPS paketi gönderildi: Lat={lat}, Lon={lon}, Alt={alt}")
                    break
            elif command.startswith('run '):
                parts = command.split()
                if len(parts) >= 2:
                    mission_id = parts[1]
                    mission_params = {}
                    if len(parts) > 2:
                        for param_str in parts[2:]:
                            if '=' in param_str:
                                key, value = param_str.split('=', 1)
                                if ',' in value:
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
                    for mid, info in drone_controller.missions.items():
                        print(f"  - ID: {mid}, İsim: {info['name']}")
                else:
                    print("Hiç görev yüklenemedi.")
            elif command == 'help':
                print_help()
            else:
                print("Bilinmeyen komut. 'help' yazarak komut listesini görebilirsiniz.")

    except Exception as e:
        print(f"Uygulama hatası: {e}")
    finally:
        await drone_controller.disconnect()
        print("Uygulama sonlandırıldı.")

if __name__ == "__main__":
    asyncio.run(main())

