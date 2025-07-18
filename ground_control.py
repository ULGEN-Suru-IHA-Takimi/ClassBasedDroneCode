import tkinter as tk
from tkinter import ttk, messagebox
import serial.tools.list_ports
import tkinter.scrolledtext as scrolledtext
from controllers.xbee_controller import XBeeController

class GroundControlApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Ground Control - XBee Port Seçimi")
        self.attributes('-zoomed', True)  # Linux için tam ekran
        self.resizable(True, True)
        self.selected_port = tk.StringVar()
        self.xbee_controller = None  # XBeeController örneği
        self.create_widgets()
        self.refresh_ports()

    def create_widgets(self):
        label = ttk.Label(self, text="XBee USB Portunu Seçin:")
        label.pack(pady=10)

        self.port_combo = ttk.Combobox(self, textvariable=self.selected_port, state="readonly")
        self.port_combo.pack(pady=5)

        refresh_btn = ttk.Button(self, text="Portları Yenile", command=self.refresh_ports)
        refresh_btn.pack(pady=5)

        connect_btn = ttk.Button(self, text="Bağlan", command=self.connect_port)
        connect_btn.pack(pady=10)

        self.remote_label = ttk.Label(self, text="Bağlı XBee cihazları:")
        self.remote_label.pack(pady=10)
        self.remote_list = tk.Listbox(self, height=5)
        self.remote_list.pack(fill=tk.X, padx=20)

        # Komut gönderme arayüzü
        self.command_label = ttk.Label(self, text="Drone Komutu Gönder:")
        self.command_label.pack(pady=10)
        self.command_entry = ttk.Entry(self)
        self.command_entry.pack(fill=tk.X, padx=20)
        self.send_command_btn = ttk.Button(self, text="Komut Gönder", command=self.send_command)
        self.send_command_btn.pack(pady=5)

        # Waypoint gönderme arayüzü
        self.wp_label = ttk.Label(self, text="Waypoint Gönder (lat,lon,alt,heading; ...):")
        self.wp_label.pack(pady=10)
        self.wp_entry = ttk.Entry(self)
        self.wp_entry.pack(fill=tk.X, padx=20)
        self.send_wp_btn = ttk.Button(self, text="Waypoint Gönder", command=self.send_waypoints)
        self.send_wp_btn.pack(pady=5)

        # Görevi Başlat butonu
        self.start_mission_btn = ttk.Button(self, text="Görevi Başlat", command=self.start_mission)
        self.start_mission_btn.pack(pady=10)

        self.log_label = ttk.Label(self, text="XBee Log Kayıtları:")
        self.log_label.pack(pady=10)
        self.log_text = scrolledtext.ScrolledText(self, height=8, state="disabled")
        self.log_text.pack(fill=tk.BOTH, padx=20, pady=5, expand=True)

    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports() if port.device.lower().endswith('usb') or 'usb' in port.device.lower()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)
        else:
            self.port_combo.set('')

    def connect_port(self):
        port = self.selected_port.get()
        if port:
            try:
                self.xbee_controller = XBeeController(port=port, baud_rate=57600)
                self.xbee_controller.device.open()
                messagebox.showinfo("Bağlantı", f"{port} portuna bağlanıldı!")
                self.list_remote_devices()
                self.xbee_controller.device.add_data_received_callback(self.listen_xbee_callback)
            except Exception as e:
                messagebox.showerror("Hata", f"Bağlantı hatası: {e}")
        else:
            messagebox.showwarning("Uyarı", "Lütfen bir port seçin!")

    def list_remote_devices(self):
        self.remote_list.delete(0, tk.END)
        try:
            network = self.xbee_controller.device.get_network()
            network.start_discovery_process()
            while network.is_discovery_running():
                self.update()
            self.remote_devices = network.get_devices()
            for dev in self.remote_devices:
                self.remote_list.insert(tk.END, str(dev.get_node_id()) + " - " + str(dev.get_64bit_addr()))
            if not self.remote_devices:
                self.remote_list.insert(tk.END, "Hiçbir XBee cihazı bulunamadı.")
        except Exception as e:
            self.remote_list.insert(tk.END, f"Hata: {e}")

    def listen_xbee_callback(self, xbee_message):
        msg = xbee_message.data.decode('utf-8')
        self.log_text.configure(state="normal")
        self.log_text.insert(tk.END, f"Gelen: {msg}\n")
        self.log_text.configure(state="disabled")
        self.log_text.see(tk.END)

    def send_command(self):
        """
        Komut girişinden alınan komutu seçili drone'a gönderir.
        """
        command = self.command_entry.get()
        selected_idx = self.remote_list.curselection()
        if not command:
            messagebox.showwarning("Uyarı", "Lütfen bir komut girin!")
            return
        if not hasattr(self, "remote_devices") or not self.remote_devices:
            messagebox.showwarning("Uyarı", "Bağlı drone yok!")
            return
        if not selected_idx:
            messagebox.showwarning("Uyarı", "Lütfen bir drone seçin!")
            return
        try:
            target_device = self.remote_devices[selected_idx[0]]
            self.xbee_controller.remote_device_cache = target_device
            self.xbee_controller.send_command(command)
            target_addr = target_device.get_64bit_addr()
            self.log_text.configure(state="normal")
            self.log_text.insert(tk.END, f"Gönderilen ({target_addr}): {command}\n")
            self.log_text.configure(state="disabled")
            self.log_text.see(tk.END)
        except Exception as e:
            messagebox.showerror("Hata", f"Komut gönderilemedi: {e}")

    def send_waypoints(self):
        """
        Kullanıcıdan alınan waypoint listesini XBee ile seçili drone'ya gönderir.
        Format: lat,lon,alt,heading; lat,lon,alt,heading; ...
        """
        wp_text = self.wp_entry.get()
        selected_idx = self.remote_list.curselection()
        if not wp_text:
            messagebox.showwarning("Uyarı", "Lütfen waypoint verisi girin!")
            return
        if not hasattr(self, "remote_devices") or not self.remote_devices:
            messagebox.showwarning("Uyarı", "Bağlı drone yok!")
            return
        if not selected_idx:
            messagebox.showwarning("Uyarı", "Lütfen bir drone seçin!")
            return
        try:
            # Waypoint stringini parse et
            waypoints = []
            for wp in wp_text.split(';'):
                parts = wp.strip().split(',')
                if len(parts) >= 4:
                    lat = float(parts[0])
                    lon = float(parts[1])
                    alt = float(parts[2])
                    heading = float(parts[3])
                    waypoints.append((lat, lon, alt, heading))
            if not waypoints:
                messagebox.showwarning("Uyarı", "Geçerli waypoint verisi yok!")
                return
            target_device = self.remote_devices[selected_idx[0]]
            self.xbee_controller.remote_device_cache = target_device
            self.xbee_controller.send_waypoints(waypoints)
            target_addr = target_device.get_64bit_addr()
            self.log_text.configure(state="normal")
            self.log_text.insert(tk.END, f"Gönderilen Waypoints ({target_addr}): {waypoints}\n")
            self.log_text.configure(state="disabled")
            self.log_text.see(tk.END)
        except Exception as e:
            messagebox.showerror("Hata", f"Waypoint gönderilemedi: {e}")

    def start_mission(self):
        """
        Seçili drone'ya XBee üzerinden 'start_mission' komutu gönderir.
        """
        selected_idx = self.remote_list.curselection()
        if not hasattr(self, "remote_devices") or not self.remote_devices:
            messagebox.showwarning("Uyarı", "Bağlı drone yok!")
            return
        if not selected_idx:
            messagebox.showwarning("Uyarı", "Lütfen bir drone seçin!")
            return
        try:
            target_device = self.remote_devices[selected_idx[0]]
            self.xbee_controller.remote_device_cache = target_device
            self.xbee_controller.send_command("start_mission")
            target_addr = target_device.get_64bit_addr()
            self.log_text.configure(state="normal")
            self.log_text.insert(tk.END, f"Gönderilen ({target_addr}): start_mission\n")
            self.log_text.configure(state="disabled")
            self.log_text.see(tk.END)
        except Exception as e:
            messagebox.showerror("Hata", f"Görev başlatılamadı: {e}")

if __name__ == "__main__":
    app = GroundControlApp()
    app.mainloop()
