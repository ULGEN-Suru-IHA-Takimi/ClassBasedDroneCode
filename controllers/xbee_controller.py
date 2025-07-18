import threading
import serial
import time
import json
from collections import deque
from digi.xbee.devices import XBeeDevice, XBee64BitAddress, RemoteXBeeDevice
from digi.xbee.exception import XBeeException, TimeoutException

DEFAULT_BAUD_RATE = 57600
SEND_INTERVAL = 0.1  # Daha hızlı iletişim için 0.1 saniye
QUEUE_RETENTION = 10  # saniye, kuyrukta tutulma süresi

class XBeePackage:
    '''
    XBee üzerinden gönderilecek/alınacak paket tanımlaması.
    Paketin 't' (type), 's' (sender) ve 'p' (parameters) alanları vardır.
    '''
    def __init__(self, package_type: str, sender: str, params: dict = None):    
        self.package_type = package_type
        self.sender = sender
        self.params = params if params is not None else {}

    def to_json(self):
        """Paketi JSON formatında bir Python sözlüğüne dönüştürür."""
        data = {
            "t": self.package_type,
            "s": self.sender,
        }
        if self.params:
            data["p"] = self.params
        return data

    def __bytes__(self):
        """Paketi JSON string'ine ve ardından UTF-8 bayt dizisine dönüştürür."""
        json_data = json.dumps(self.to_json())
        encoded_data = json_data.encode('utf-8')
        # Paket boyutu uyarısı send_package metodunda daha detaylı ele alınacak.
        return encoded_data

    def __str__(self):
        return f"Type:{self.package_type}, Sender:{self.sender}, Params:{self.params}"

    @classmethod
    def from_bytes(cls, byte_data):
        """Bayt dizisinden XBeePackage nesnesi oluşturur."""
        decoded_data = byte_data.decode('utf-8')
        json_data = json.loads(decoded_data)

        package_type = json_data.get("t")
        sender = json_data.get("s")
        params = json_data.get("p", {})

        return cls(package_type, sender, params)


class XBeeController:
    def __init__(self, port="/dev/ttyUSB0"):
        self.device = None
        self.received_queue = deque()  # Gelen paket sırası
        self.send_queue = deque()  # Paket gönderme sırası
        self.queue_lock = threading.Lock() # Kuyruk erişimi için kilit
        self.local_xbee_address: XBee64BitAddress = None
        self.send_interval: float = SEND_INTERVAL

        # İç thread'ler
        self.cleaner_thread = None
        self.sender_thread = None
        self._stop_event = threading.Event() # Thread'leri durdurmak için event

        self.connected = False

        try:
            self.device = XBeeDevice(port, DEFAULT_BAUD_RATE)
            self.device.open()
            print("[Xbee Controller]: Xbee cihazı açıldı.")
            self.device.add_data_received_callback(self._receive_data_callback)
            self.connected = True
            self._start_internal_threads() # Sadece bağlantı başarılıysa thread'leri başlat

        except serial.SerialException as e:
            print(f"[Xbee Controller]: Hata! Seri porta bağlanılamadı: {e}")
        except XBeeException as e:
            print(f"[Xbee Controller]: Hata! XBee cihaza bağlanılamadı veya yapılandırılamadı: {e}")
        except Exception as e:
            print(f"[Xbee Controller]: Beklenmedik bir hata oluştu: {e}")

    def disconnect(self):
        """XBee bağlantısını kapatır ve iç thread'leri durdurur."""
        if self.connected:
            print("[Xbee Controller]: Xbee cihazı kapatılıyor...")
            self._stop_event.set() # Thread'lere durma sinyali gönder
            
            # Thread'lerin bitmesini bekle
            if self.cleaner_thread and self.cleaner_thread.is_alive():
                self.cleaner_thread.join(timeout=1)
            if self.sender_thread and self.sender_thread.is_alive():
                self.sender_thread.join(timeout=1)

            if self.device and self.device.is_open():
                self.device.close()
            print("[Xbee Controller]: Xbee cihazı kapatıldı.")
        else:
            print("[Xbee Controller]: Xbee cihazı zaten kapalı.")
        
        self.device = None
        self.local_xbee_address = None
        self.connected = False

    def _start_internal_threads(self):
        """Modülün iç thread'lerini (gönderici ve temizleyici) başlatır."""
        if not self.cleaner_thread or not self.cleaner_thread.is_alive():
            self.cleaner_thread = threading.Thread(target=self._clean_queues_loop, name="XBeeCleanerThread", daemon=True)
            self.cleaner_thread.start()
            print("[Xbee Controller]: Temizleyici thread başlatıldı.")

        if not self.sender_thread or not self.sender_thread.is_alive():
            self.sender_thread = threading.Thread(target=self._send_loop, name="XBeeSenderThread", daemon=True)
            self.sender_thread.start()
            print("[Xbee Controller]: Gönderici thread başlatıldı.")

    def _clean_queues_loop(self):
        """Kuyrukları periyodik olarak temizler."""
        while not self._stop_event.is_set():
            now = time.time()
            with self.queue_lock:
                # Gönderme kuyruğunu temizle
                while self.send_queue and (now - self.send_queue[0][0]) > QUEUE_RETENTION:
                    self.send_queue.popleft()
                # Alma kuyruğunu temizle (varsa)
                while self.received_queue and (now - self.received_queue[0][0]) > QUEUE_RETENTION:
                    self.received_queue.popleft()
            self._stop_event.wait(1) # Her saniye kontrol et

    def receive(self):
        """
        Kuyruktan gelen ilk paketi okur ve döndürür.
        Eğer kuyruk boşsa None döner.
        """
        with self.queue_lock:
            if self.received_queue:
                # Kuyrukta saklanan format: (timestamp, package_json_dict)
                timestamp, package_data = self.received_queue.popleft()
                return package_data
            else:
                return None

    def send(self, package: XBeePackage, remote_xbee_addr_hex: str = None):
        """
        Belirtilen XBeePackage nesnesini gönderim kuyruğuna ekler.
        Gönderim işlemi arka plandaki _send_loop tarafından yönetilir.
        :param package: Gönderilecek XBeePackage nesnesi.
        :param remote_xbee_addr_hex: Hedef XBee'nin 64-bit adresi (hex string olarak).
                                  Sadece API modunda kullanılır. Broadcast için "000000000000FFFF".
        """
        if not self.connected:
            print("[XBee Controller]: Cihaz bağlı değil, paket gönderilemiyor.")
            return

        with self.queue_lock:
            self.send_queue.append((time.time(), package, remote_xbee_addr_hex))
            print(f"Paket gönderim kuyruğuna eklendi. Tipi: {package.package_type}")

    def _receive_data_callback(self, xbee_message):
        """XBee cihazından veri geldiğinde çağrılan geri çağırma fonksiyonu."""
        data = xbee_message.data
        remote_address_64bit = None
        if hasattr(xbee_message, 'remote_device') and xbee_message.remote_device:
            try:
                remote_address_64bit = xbee_message.remote_device.get_64bit_addr().address.hex()
                print(f"Paket geldi: Boyut={len(data)}, Kaynak={remote_address_64bit}")
            except Exception as e:
                print(f"Uyarı: Uzak cihaz adres bilgisi alınamadı (geri çağırma içinde): {e}")
                pass
        try:
            received_package = XBeePackage.from_bytes(data)
            with self.queue_lock:
                self.received_queue.append((time.time(), received_package.to_json()))
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            with self.queue_lock:
                self.received_queue.append((time.time(), {"error": str(e), "raw_data_hex": data.hex(), "source_addr": remote_address_64bit}))
        except Exception as e:
            with self.queue_lock:
                self.received_queue.append((time.time(), {"error": "Genel İşleme Hatası: " + str(e), "source_addr": remote_address_64bit}))

    def _send_loop(self):
        """Arka planda gönderim kuyruğundaki paketleri periyodik olarak gönderir."""
        while not self._stop_event.is_set():
            package_to_send = None
            with self.queue_lock:
                if self.send_queue:
                    package_time, package, remote_xbee_addr_hex = self.send_queue.popleft()
                    package_to_send = (package, remote_xbee_addr_hex)

            if package_to_send:
                self._do_send(package_to_send[0], package_to_send[1])
            
            self._stop_event.wait(self.send_interval) # Belirtilen aralıklarla bekle

        print("[Xbee Controller]: XBee Sender Thread durduruldu.")

    def _do_send(self, package: XBeePackage, remote_xbee_addr_hex: str = None):
        """Paket gönderme işlemini gerçekleştirir."""
        data_to_send = bytes(package)
        
        # Maksimum payload 65 byte.
        if len(data_to_send) > 65:
            print(f"UYARI: Gönderilmek istenen paket boyutu ({len(data_to_send)} bayt) çok büyük. Paket kısmen veya hiç gönderilemeyebilir.")

        try:
            if remote_xbee_addr_hex:
                remote_addr_obj = XBee64BitAddress(bytes.fromhex(remote_xbee_addr_hex))
                remote_xbee = RemoteXBeeDevice(self.device, remote_addr_obj)
                self.device.send_data(remote_xbee, data_to_send)
                print(f"Paket API modunda gönderildi: Tipi='{package.package_type}', Hedef='{remote_xbee_addr_hex}', Boyut={len(data_to_send)} bayt")
            else:
                self.device.send_data_broadcast(data_to_send)
                print(f"Paket API modunda BROADCAST edildi: Tipi='{package.package_type}', Boyut={len(data_to_send)} bayt")

        except TimeoutException:
            print(f"Hata: Paket gönderilirken zaman aşımı oluştu. Hedef XBee ulaşılamıyor olabilir.")
        except XBeeException as e:
            print(f"Hata: XBee gönderme hatası: {e}")
        except Exception as e:
            print(f"Beklenmedik bir hata oluştu paket gönderilirken: {e}")


if __name__ == "__main__":
    print('XBee bağlantısı için port girin (örn: 0, 1, ... veya "default" için boş bırakın)')
    input_port_str = input('/dev/ttyUSB? : ')

    if input_port_str.strip() == "":
        xbee_port = "/dev/ttyUSB0"
    else:
        try:
            # Sadece sayıyı alıp /dev/ttyUSBX formatına dönüştürüyoruz
            port_number = int(input_port_str)
            xbee_port = f"/dev/ttyUSB{port_number}"
        except ValueError:
            print("Geçersiz port girişi. Varsayılan olarak /dev/ttyUSB0 kullanılacak.")
            xbee_port = "/dev/ttyUSB0"

    my_xbee = XBeeController(port=xbee_port)

    # XBee cihazına bağlanılamazsa uygulamayı sonlandır
    if not my_xbee.connected:
        print("XBee cihaza bağlanılamadığı için uygulama sonlandırılıyor.")
        exit()

    def process_package_loop():
        """Gelen paketleri sürekli olarak işleyen döngü."""
        while my_xbee.connected:
            incoming_package_json = my_xbee.receive()
            if incoming_package_json:
                print("\n--- Gelen Paket İşleniyor... ---")
                if "error" in incoming_package_json:
                    print(f"  Paket işleme hatası: {incoming_package_json['error']}")
                    if "raw_data_hex" in incoming_package_json:
                        print(f"  Ham Veri (Hex): {incoming_package_json['raw_data_hex']}")
                    if "source_addr" in incoming_package_json:
                        print(f"  Kaynak Adres: {incoming_package_json['source_addr']}")
                else:
                    print(f"  Tip: {incoming_package_json.get('t')}")
                    print(f"  Gönderen: {incoming_package_json.get('s')}")
                    print(f"  Parametreler: {incoming_package_json.get('p')}")

                    package_type = incoming_package_json.get('t')
                    sender_id = incoming_package_json.get('s')
                    params = incoming_package_json.get('p', {})
                    if package_type == "G":
                        latitude = params.get('x') / 1000000.0 if params.get('x') is not None else "N/A"
                        longitude = params.get('y') / 1000000.0 if params.get('y') is not None else "N/A"
                        print(f"    GPS verisi alındı ve işlendi: Gönderen={sender_id}, Lat={latitude}, Lon={longitude}")
                    elif package_type == "H":
                        print(f"    El sıkışma alındı: Gönderen={sender_id}")
                    elif package_type == "W":
                        waypoint_id = sender_id
                        latitude = params.get('x') / 1000000.0 if params.get('x') is not None else "N/A"
                        longitude = params.get('y') / 1000000.0 if params.get('y') is not None else "N/A"
                        heading = params.get('h', 0) # Eğer heading pakette geliyorsa
                        print(f"    Waypoint alındı: ID={waypoint_id}, Lat={latitude}, Lon={longitude}, Heading={heading}")
                    elif package_type == "w":
                        waypoint_id = sender_id
                        print(f"    Waypoint kaldırma isteği alındı: ID={waypoint_id}")
                    elif package_type == "O":
                        print(f"    Görev için emir/order geldi: Görev id={sender_id}, Parametreler={params}")
                    elif package_type == "MC":
                        print(f"    Göreve başlama onayı geldi: Gönderen={sender_id}, Görev numarası={params.get('id', 'N/A')}")
                    else:
                        print(f"    Bilinmeyen paket tipi alındı: {package_type}")
            else:
                time.sleep(0.05) # Kuyruk boşsa kısa bir süre bekle

    def send_dummy_data_loop():
        """Belirli aralıklarla test paketi gönderen döngü."""
        counter = 0
        while my_xbee.connected:
            # Farklı paket tipleri gönderebilirsiniz
            my_xbee.send(XBeePackage("G", "1", {"x": int(47.397606 * 1000000) + counter, "y": int(8.543060 * 1000000)}))
            # my_xbee.send(XBeePackage("H", "Heartbeat_Node1"))
            # my_xbee.send(XBeePackage("W", "WP_1", {"x": 47.400000*1000000, "y": 8.550000*1000000, "h": 90}))
            
            counter += 1
            time.sleep(5) # Her 5 saniyede bir paket gönder

    # İşleme ve gönderme thread'lerini başlat
    process_package_thread = threading.Thread(target=process_package_loop, name="ProcessPackageThread", daemon=True)
    process_package_thread.start()

    send_dummy_data_thread = threading.Thread(target=send_dummy_data_loop, name="SendDummyDataThread", daemon=True)
    send_dummy_data_thread.start()

    try:
        input("Kapatmak için enter'a basın...\n")
    except KeyboardInterrupt:
        print("\nÇıkış isteği alındı.")
    finally:
        my_xbee.disconnect()
        print("Uygulama sonlandırıldı.")