import os
import sys
import struct
import queue
import threading
import importlib
import pkgutil
import serial
import serial.tools.list_ports
import customtkinter as ctk

# --- Theme Configuration ---
ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")

PACKET_FORMAT = '<6fB'  # 6 floats (X, Y, Z, TiltX, TiltY, Yaw) + 1 byte XOR checksum
PACKET_SIZE = struct.calcsize(PACKET_FORMAT) # 25 bytes following magic header 0xAA

class HexaControlApp(ctk.CTk):
    def __init__(self):
        super().__init__()

        self.title("HexaControl")
        self.geometry("380x600")
        self.resizable(False, False)

        # State variables
        self.serial_port = None
        self.is_connected = False
        self.packet_queue = queue.Queue()
        self.plugins = {}  # {plugin_name: (module_instance, var_enabled)}

        # UI Layout
        self._build_ui()

        # Start background serial worker & UI poller
        self.worker_thread = threading.Thread(target=self._serial_worker, daemon=True)
        self.worker_thread.start()

        self.load_plugins()
        self._poll_queue()

    def _build_ui(self):
        # Main container with compact padding
        main_frame = ctk.CTkFrame(self, fg_color="transparent")
        main_frame.pack(fill="both", expand=True, padx=15, pady=15)

        # --- Top Section: Connection Header ---
        conn_frame = ctk.CTkFrame(main_frame, fg_color="#2b2b2b", corner_radius=8)
        conn_frame.pack(fill="x", padx=0, pady=(0, 10))

        # Row 1: Dropdown & Connect Button
        top_row = ctk.CTkFrame(conn_frame, fg_color="transparent")
        top_row.pack(fill="x", padx=12, pady=12)

        active_ports = self._get_active_serial_ports()
        self.port_dropdown = ctk.CTkComboBox(top_row, values=active_ports if active_ports else ["No Active Ports"], width=190, height=32)
        self.port_dropdown.pack(side="left", padx=(0, 8))

        self.connect_btn = ctk.CTkButton(top_row, text="Connect", width=95, height=32, fg_color="#22c55e", hover_color="#16a34a", font=("Helvetica", 12, "bold"), command=self._toggle_connection)
        self.connect_btn.pack(side="right")

        # Row 2: Status Indicator & Recalibrate Button
        bottom_row = ctk.CTkFrame(conn_frame, fg_color="transparent")
        bottom_row.pack(fill="x", padx=12, pady=(0, 12))

        status_container = ctk.CTkFrame(bottom_row, fg_color="transparent")
        status_container.pack(side="left")

        self.status_canvas = ctk.CTkCanvas(status_container, width=16, height=16, bg="#2b2b2b", highlightthickness=0)
        self.status_canvas.pack(side="left", padx=(0, 6))
        self.status_led = self.status_canvas.create_oval(2, 2, 14, 14, fill="#ef4444")

        self.status_lbl = ctk.CTkLabel(status_container, text="Disconnected", font=("Helvetica", 11), text_color="#a3a3a3")
        self.status_lbl.pack(side="left")

        self.calib_btn = ctk.CTkButton(bottom_row, text="Recalibrate", width=110, height=28, fg_color="#f59e0b", hover_color="#d97706", font=("Helvetica", 11, "bold"), command=self._send_calibration)
        self.calib_btn.pack(side="right")

        settings_outer = ctk.CTkFrame(main_frame, fg_color="#2b2b2b", corner_radius=8)
        settings_outer.pack(fill="x", padx=0, pady=(0, 10))
        ctk.CTkLabel(settings_outer, text="Settings", font=("Helvetica", 12, "bold"), text_color="#d4d4d4").pack(padx=12, pady=(10, 4), anchor="w")
        self.plugin_settings = ctk.CTkFrame(settings_outer, fg_color="transparent")
        self.plugin_settings.pack(fill="x", padx=4, pady=(0, 4))

        # --- Bottom Section: Plugin List ---
        plugin_outer = ctk.CTkFrame(main_frame, fg_color="#2b2b2b", corner_radius=8)
        plugin_outer.pack(fill="both", expand=True, padx=0, pady=0)

        ctk.CTkLabel(plugin_outer, text="Loaded Plugins", font=("Helvetica", 12, "bold"), text_color="#d4d4d4").pack(padx=12, pady=(12, 6), anchor="w")

        self.plugin_scroll = ctk.CTkScrollableFrame(plugin_outer, fg_color="transparent", height=380)
        self.plugin_scroll.pack(fill="both", expand=True, padx=8, pady=(0, 8))

    def _get_active_serial_ports(self):
        """Scans and filters strictly for active connected physical serial ports."""
        ports = []
        for p in serial.tools.list_ports.comports():
            # Filter out virtual/dummy bluetooth links if needed, keeping physical USB devices (ACM / USB / COM)
            if any(k in p.device.lower() for k in ["ttyacm", "ttyusb", "com"]):
                ports.append(p.device)
            elif p.hwid and "n/a" not in p.hwid.lower():
                ports.append(p.device)
        return ports

    def _toggle_connection(self):
        if not self.is_connected:
            port = self.port_dropdown.get()
            if not port or port in ["No Active Ports", ""]:
                return
            try:
                self.serial_port = serial.Serial(port, 115200, timeout=0.01)
                self.is_connected = True
                self.connect_btn.configure(text="Disconnect", fg_color="#ef4444", hover_color="#dc2626")
                self.status_canvas.itemconfig(self.status_led, fill="#22c55e")
                self.status_lbl.configure(text="Connected", text_color="#22c55e")
            except Exception as e:
                print(f"[Connection Error]: {e}")
        else:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            self.is_connected = False
            self.connect_btn.configure(text="Connect", fg_color="#22c55e", hover_color="#16a34a")
            self.status_canvas.itemconfig(self.status_led, fill="#ef4444")
            self.status_lbl.configure(text="Disconnected", text_color="#a3a3a3")

    def _send_calibration(self):
        if self.is_connected and self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(b'n\n')
            except Exception as e:
                print(f"[Calibration Error]: {e}")

    def load_plugins(self):
        plugins_dir = "plugins"
        if not os.path.exists(plugins_dir):
            os.makedirs(plugins_dir)

        for _, module_name, _ in pkgutil.iter_modules([plugins_dir]):
            try:
                mod = importlib.import_module(f"plugins.{module_name}")
                if hasattr(mod, "Plugin"):
                    plugin_instance = mod.Plugin()
                    if hasattr(plugin_instance, "start"):
                        plugin_instance.start()
                    var = ctk.BooleanVar(value=False)
                    self.plugins[module_name] = (plugin_instance, var)

                    # Sleek compact plugin list item row
                    row = ctk.CTkFrame(self.plugin_scroll, fg_color="#1f1f1f", corner_radius=6, height=40)
                    row.pack(fill="x", padx=4, pady=4)
                    row.pack_propagate(False)

                    ctk.CTkLabel(row, text=module_name.replace("_", " ").title(), font=("Helvetica", 11, "bold"), text_color="#e5e5e5").pack(side="left", padx=10)

                    switch = ctk.CTkSwitch(row, text="", variable=var, width=40, progress_color="#22c55e")
                    switch.pack(side="right", padx=10)

                    if hasattr(plugin_instance, "build_settings"):
                        plugin_instance.build_settings(self.plugin_settings)

            except Exception as e:
                print(f"[Plugin Load Error ({module_name})]: {e}")

    def _shutdown_plugins(self):
        for name, (instance, _) in self.plugins.items():
            if hasattr(instance, "stop"):
                try:
                    instance.stop()
                except Exception as e:
                    print(f"[Plugin Shutdown Error ({name})]: {e}")
        self.destroy()

    def _serial_worker(self):
        while True:
            if self.is_connected and self.serial_port:
                try:
                    header = self.serial_port.read(1)
                    if header and header[0] == 0xAA:
                        payload_bytes = self.serial_port.read(PACKET_SIZE)
                        if len(payload_bytes) < PACKET_SIZE:
                            continue

                        raw_data = payload_bytes[:24]
                        received_crc = payload_bytes[24]
                        calc_crc = 0
                        for b in raw_data:
                            calc_crc ^= b

                        if calc_crc != received_crc:
                            continue

                        floats = struct.unpack('<6f', raw_data)
                        packet = {
                            'X': floats[0], 'Y': floats[1], 'Z': floats[2],
                            'TiltX': floats[3], 'TiltY': floats[4], 'Yaw': floats[5]
                        }

                        self.packet_queue.put(packet)
                except (serial.SerialException, OSError):
                    self.is_connected = False
                    self.connect_btn.configure(text="Connect", fg_color="#22c55e", hover_color="#16a34a")
                    self.status_canvas.itemconfig(self.status_led, fill="#ef4444")
                    self.status_lbl.configure(text="Disconnected", text_color="#a3a3a3")
            else:
                import time
                time.sleep(0.05)

    def _poll_queue(self):
        try:
            while True:
                packet = self.packet_queue.get_nowait()
                for name, (instance, var) in self.plugins.items():
                    if var.get() and hasattr(instance, "on_packet"):
                        try:
                            instance.on_packet(packet)
                        except Exception as e:
                            print(f"[Plugin Error ({name})]: {e}")
        except queue.Empty:
            pass

        self.after(20, self._poll_queue)

if __name__ == "__main__":
    app = HexaControlApp()
    app.protocol("WM_DELETE_WINDOW", app._shutdown_plugins)
    app.mainloop()
