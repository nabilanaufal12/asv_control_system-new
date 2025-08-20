# gui/api_client.py
# --- MODIFIKASI FINAL: Beralih sepenuhnya ke Klien WebSocket ---

import threading
import socketio
from PySide6.QtCore import QObject, Signal, Slot

class ApiClient(QObject):
    """
    Kelas ini sekarang bertindak sebagai Klien WebSocket untuk komunikasi
    real-time dengan Backend Server. Ia tidak lagi menggunakan HTTP polling.
    """
    data_updated = Signal(dict)
    connection_status_changed = Signal(bool, str)

    def __init__(self, config):
        super().__init__()
        backend_config = config.get("backend_connection", {})
        self.base_url = f"http://{backend_config.get('ip_address', '127.0.0.1')}:{backend_config.get('port', 5000)}"
        
        # 1. Buat instance klien Socket.IO
        self.sio = socketio.Client()
        
        # 2. Siapkan event handler (apa yang harus dilakukan saat menerima pesan)
        self.setup_event_handlers()

        # 3. Jalankan koneksi di thread terpisah agar tidak memblokir GUI
        self.conn_thread = threading.Thread(target=self.connect_to_server, daemon=True)
        self.conn_thread.start()

    def connect_to_server(self):
        """Mencoba terhubung dan tetap terhubung ke server WebSocket."""
        print(f"ApiClient mencoba terhubung ke server WebSocket di {self.base_url}")
        try:
            # Menghubungkan ke server
            self.sio.connect(self.base_url)
            # baris sio.wait() akan memblokir thread ini, membuatnya tetap hidup
            # untuk mendengarkan pesan dari server.
            self.sio.wait()
        except socketio.exceptions.ConnectionError:
            print("Koneksi ke server WebSocket gagal.")
            self.connection_status_changed.emit(False, "Backend tidak terjangkau")
        print("Thread koneksi ApiClient dihentikan.")

    def setup_event_handlers(self):
        """
        Mendefinisikan fungsi (callback) yang akan dijalankan saat 
        menerima event tertentu dari server.
        """
        @self.sio.event
        def connect():
            # Saat berhasil terhubung
            self.connection_status_changed.emit(True, "Terhubung ke Backend")
            print("Berhasil terhubung ke server WebSocket!")

        @self.sio.event
        def disconnect():
            # Saat koneksi terputus
            self.connection_status_changed.emit(False, "Koneksi ke Backend terputus")
            print("Koneksi ke server terputus.")

        @self.sio.on('telemetry_update')
        def on_telemetry_update(data):
            # Saat server mengirim event 'telemetry_update'
            # Kita teruskan datanya ke seluruh GUI melalui sinyal Qt
            self.data_updated.emit(data)

    def _send_command(self, command_name, payload_data):
        """Fungsi helper untuk mengirim perintah ke backend via event 'command'."""
        if self.sio.connected:
            self.sio.emit('command', {'command': command_name, 'payload': payload_data})
        else:
            print("Tidak bisa mengirim perintah, tidak terhubung ke server.")

    # --- KUMPULAN SLOT UNTUK MENANGANI SINYAL DARI GUI ---
    # Slot-slot ini sekarang akan memanggil _send_command via WebSocket.

    @Slot(dict)
    def connect_to_port(self, connection_details):
        self._send_command("CONFIGURE_SERIAL", connection_details)

    @Slot(str)
    def handle_mode_change(self, mode):
        self._send_command("CHANGE_MODE", mode)

    @Slot(list)
    def handle_manual_keys(self, keys):
        self._send_command("MANUAL_CONTROL", keys)

    @Slot(list)
    def set_waypoints(self, waypoints):
        self._send_command("SET_WAYPOINTS", waypoints)

    @Slot(dict)
    def handle_vision_status(self, vision_data):
        self._send_command("VISION_OVERRIDE", vision_data)

    def shutdown(self):
        """Memutuskan koneksi saat aplikasi ditutup."""
        print("Memutuskan koneksi WebSocket...")
        if self.sio.connected:
            self.sio.disconnect()