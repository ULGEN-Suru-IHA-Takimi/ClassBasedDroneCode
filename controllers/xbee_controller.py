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
    def __init__(self,port="/dev/ttyUSB0"):
        self.device = XBeeDevice(port, DEFAULT_BAUD_RATE)
        self.received_queue = deque() # Gelen paket sırası
        self.send_queue = deque() # Paket gönderme sırası
        self.queue_lock = threading.Lock()

        self.local_xbee_address: XBee64BitAddress = None
        self.send_interval: float = 1.0
        # İç thread'ler
        self.cleaner_thread = None
        self.sender_thread = None
        self.receiver_callback_set = False
        self.connected = False

        try:
            self.device.open()
            print("[Xbee Controller]: Xbee cihazı açıldı.")
            if not self.receiver_callback_set:
                self.device.add_data_received_callback(self._receive_data_callback)
                self.receiver_callback_set = True

        except serial.SerialException as e:
            print(f"[Xbee Controller]: Hata! Seri porta bağlanılamadı: {e}")
        except XBeeException as e:
            print(f"[Xbee Controller]: Hata! XBee cihaza bağlanılamadı veya yapılandırılamadı: {e}")
        except Exception as e:
            print(f"[Xbee Controller]: Beklenmedik bir hata oluştu: {e}")
        finally:
            self.connected = self.device.is_open()
            self._start_internal_threads()
    # Bağlantıyı kapat
    def disconnect(self):
        if self.device or self.connected:
            self.device.close()
            print("[Xbee Controller]: Xbee cihazı kapatıldı.")
        else:
            print("[Xbee Controller]: Xbee cihazı zaten kapalı.")
        self.device = None
        self.local_xbee_address = None
        self.receiver_callback_set = False
        self.connected = False

    # arkaplan fonksiyonlarını çalıştır
    def _start_internal_threads(self):
        """Modülün iç thread'lerini (gönderici ve temizleyici) başlatır."""
        if not self.cleaner_thread or not self.cleaner_thread.is_alive():
            self.cleaner_thread = threading.Thread(target=self._clean_queues_loop, name="XBeeCleanerThread", daemon=True)
            self.cleaner_thread.start()
        
        if not self.sender_thread or not self.sender_thread.is_alive():
            self.sender_thread = threading.Thread(target=self._send_loop, name="XBeeSenderThread", daemon=True)
            self.sender_thread.start()

    # Sıraları temizle
    def _clean_queues_loop(self):
        while self.device and self.connected:
            now = time.time()
            with self.queue_lock:
                while self.send_queue and (now - self.send_queue[0][0]) > QUEUE_RETENTION:
                    self.send_queue.popleft()

    # Gelen paket listesindeki ilk gelen paketi okuma komutu
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

    #Paketi göndermek için gönderme sırasına ekleme komutu
    def send(self, package: XBeePackage, remote_xbee_addr_hex: str = None):
        """
        Belirtilen XBeePackage nesnesini gönderim kuyruğuna ekler.
        Gönderim işlemi arka plandaki _send_loop tarafından yönetilir.
        :param package: Gönderilecek XBeePackage nesnesi.
        :param remote_xbee_addr_hex: Hedef XBee'nin 64-bit adresi (hex string olarak).
                                  Sadece API modunda kullanılır. Broadcast için "000000000000FFFF".
        """
        with self.queue_lock:
            self.send_queue.append((time.time(),package, remote_xbee_addr_hex))


    # Arkadan sürekli paketi alan loop
    def _receive_data_callback(self, xbee_message):
        data = xbee_message.data
        remote_address_64bit = None
        if hasattr(xbee_message, 'remote_device') and xbee_message.remote_device:
            try:
                remote_address_64bit = xbee_message.remote_device.get_64bit_addr().address.hex() 
                print(f"Paket geldi: Boyut={len(data)}")
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
            # print(f"Hata: Gelen paket işlenirken beklenmedik sorun oluştu: {e}")
            with self.queue_lock:
                self.received_queue.append((time.time(), {"error": "Genel İşleme Hatası: " + str(e), "source_addr": remote_address_64bit}))


    # Arkada çalışan paket gönderme loopu
    def _send_loop(self):
        """Arka planda gönderim kuyruğundaki paketleri periyodik olarak gönderir."""
        while self.device and self.connected:
            with self.queue_lock:
                if self.send_queue:
                    package_time, package, remote_xbee_addr_hex = self.send_queue.popleft()
                    self._do_send(package, remote_xbee_addr_hex)
            time.sleep(self.send_interval)
        print("[Xbee Controller]: XBee Sender Thread durduruldu.")

    # Paketi gönderme fonksiyonu
    def _do_send(self, package: XBeePackage, remote_xbee_addr_hex: str = None):
        """Paket gönderme işlemini gerçekleştirir."""
        data_to_send = bytes(package)
        print(f"Paket gönderildi: boyut={len(data_to_send)}")
        
        # Maksimum payload 65 byte.
        if len(data_to_send) > 65: 
            print(f"UYARI: Gönderilmek istenen paket boyutu ({len(data_to_send)} bayt)")

        try:
            if remote_xbee_addr_hex:
                remote_addr_obj = XBee64BitAddress(bytes.fromhex(remote_xbee_addr_hex)) 
                remote_xbee = RemoteXBeeDevice(self.device, remote_addr_obj)
                self.device.send_data(remote_xbee, data_to_send)
                # print(f"Paket API modunda gönderildi: Tipi='{package.package_type}', Hedef='{remote_xbee_addr_hex}', Boyut={len(data_to_send)} bayt")
            else:
                self.device.send_data_broadcast(data_to_send)
                # print(f"Paket API modunda BROADCAST edildi: Tipi='{package.package_type}', Boyut={len(data_to_send)} bayt")
            
        except TimeoutException:
            print(f"Hata: Paket gönderilirken zaman aşımı oluştu. Hedef XBee ulaşılamıyor olabilir.")
        except XBeeException as e:
            print(f"Hata: XBee gönderme hatası: {e}")
        except Exception as e:
            print(f"Beklenmedik bir hata oluştu paket gönderilirken: {e}")


if __name__ == "__main__":
    print('XBee bağlantısı için port girin ("q" default)')
    input_port = input('/dev/ttyUSB? :')

    if input_port == "q":
        my_xbee = XBeeController(port="/dev/ttyUSB"+input_port)
    else:
        my_xbee = XBeeController()

    def process_package():
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
                        # Waypoint ekleme/güncelleme fonksiyonu
                    elif package_type == "w":
                        # Waypoint kaldırma fonksiyonu
                        waypoint_id = sender_id
                        pass
                    elif package_type == "O":
                        print(f"    Görev için emir/order geldi: Görev id={sender_id}, Parametreler={params}")
                    elif package_type == "MC":
                        print(f"    Göreve başlama onayı geldi: Gönderen={sender_id}, Görev numarası={params.get('id', 'N/A')}")
                    else:
                        print(f"    Bilinmeyen paket tipi alındı: {package_type}")

    def send_loop():
        while my_xbee.connected:
            my_xbee.send(XBeePackage("G","1",{"x":47.397606*1000000, "y":8.543060*1000000}))
            time.sleep(5)
    

    process_package_thread = threading.Thread(target=process_package, name="ProcessPackageThread", daemon=True)
    process_package_thread.start()

    send_loop_thread = threading.Thread(target=send_loop, name="SendLoopThread", daemon=True)
    send_loop_thread.start()

    input("Kapatmak için enter`a basın...")

    my_xbee.disconnect()