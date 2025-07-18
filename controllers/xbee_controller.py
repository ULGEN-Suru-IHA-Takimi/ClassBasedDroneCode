import threading
import time
from collections import deque
from digi.xbee.devices import XBeeDevice

class XBeeController:
    def __init__(self, port="/dev/ttyUSB0", baud_rate=57600, send_interval=0.1, queue_retention=10, remote_node_id="REMOTE", drone_id=None):
        self.port = port
        self.baud_rate = baud_rate
        self.send_interval = send_interval
        self.queue_retention = queue_retention
        self.remote_node_id = remote_node_id
        self.drone_id = drone_id
        self.device = XBeeDevice(self.port, self.baud_rate)
        self.remote_device_cache = None
        self.signal_queue = deque()
        self.queue_lock = threading.Lock()
        self.running = True

    # Gönderilecek mesaj (kısa, virgül ile ayrılmış string)
 

    def get_command(self, command = "takeoff"):
        prefix = f"{self.drone_id}," if self.drone_id else ""
        cmd = f"{prefix}{command}"
        return self._truncate_message(cmd)

    def get_gps(self, lat=47.397742, lon=8.545594, alt=None):
        prefix = f"{self.drone_id}," if self.drone_id else ""
        if alt is not None:
            gps = f"{prefix}{lat*1000000},{lon*1000000},{alt}"
        else:
            gps = f"{prefix}{lat*1000000},{lon*1000000}"
        return self._truncate_message(gps)

    
    def _truncate_message(self, msg):
        # XBee'den gelen/giden veri boyutu 65 KB'ı aşmayacak şekilde ayarlanır
        max_bytes = 65 * 1024  # 65 KB
        encoded = msg.encode('utf-8')
        if len(encoded) > max_bytes:
            print(f"⚠️ Mesaj boyutu {len(encoded)} bayt, 65 KB sınırını aşıyor. Kısaltılıyor.")
            return encoded[:max_bytes].decode('utf-8', errors='ignore')
        return msg

    def data_receive_callback(self, xbee_message):
        try:
            data = xbee_message.data.decode('utf-8')
            fields = data.split(',')
            print(f"📩 Gelen mesaj: {fields}")
        except Exception as e:
            print(f"📩 Gelen mesaj (ham): {xbee_message.data} (Hata: {e})")
            fields = xbee_message.data
        with self.queue_lock:
            self.signal_queue.append((time.time(), 'IN', fields))

    def send_data_periodically(self):
        if self.remote_node_id:
            try:
                self.remote_device_cache = self.device.get_network().discover_device(self.remote_node_id)
                if self.remote_device_cache:
                    print(f"🎯 Uzak cihaz bulundu: {self.remote_device_cache.get_64bit_addr()}")
                else:
                    print("⚠️ Uzak cihaz bulunamadı, broadcast ile gönderilecek.")
            except Exception as e:
                print(f"Cihaz bulma hatası: {e}")
        while self.device.is_open() and self.running:
            try:
                msg = self.get_message()
                if self.remote_device_cache:
                    self.device.send_data(self.remote_device_cache, msg)
                    print(f"✅ Mesaj gönderildi. ({len(msg.encode('utf-8'))} bayt)")
                else:
                    self.device.send_data_broadcast(msg)
                    print(f"📡 Broadcast ile mesaj gönderildi. ({len(msg.encode('utf-8'))} bayt)")
                with self.queue_lock:
                    self.signal_queue.append((time.time(), 'OUT', msg))
            except Exception as e:
                print(f"Gönderim hatası: {e}")
            time.sleep(self.send_interval)

    def queue_cleaner(self):
        while self.running:
            now = time.time()
            with self.queue_lock:
                while self.signal_queue and now - self.signal_queue[0][0] > self.queue_retention:
                    self.signal_queue.popleft()
            time.sleep(1)

    def get_last_received(self):
        with self.queue_lock:
            for t, direction, data in reversed(self.signal_queue):
                if direction == 'IN':
                    return data
        return None

    def get_last_sent(self):
        with self.queue_lock:
            for t, direction, data in reversed(self.signal_queue):
                if direction == 'OUT':
                    return data
        return None

    def run(self):
        try:
            self.device.open()
            print("📡 Dinleniyor ve gönderiyor...")
            self.device.add_data_received_callback(self.data_receive_callback)

            sender_thread = threading.Thread(target=self.send_data_periodically, daemon=True)
            sender_thread.start()

            cleaner_thread = threading.Thread(target=self.queue_cleaner, daemon=True)
            cleaner_thread.start()

            input("Çıkmak için Enter'a basın...\n")
            self.running = False
        finally:
            if self.device.is_open():
                self.device.close()
                print("🔌 Bağlantı kapatıldı.")

if __name__ == "__main__":
    controller = XBeeController()
    controller.run()