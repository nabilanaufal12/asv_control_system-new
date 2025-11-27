# src/navantara_gui/components/map_view.py
from PySide6.QtWidgets import QWidget, QVBoxLayout
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtCore import QUrl
import json


class MapView(QWidget):
    def __init__(self, config=None):
        super().__init__()
        self.config = config
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.web_view = QWebEngineView()
        self.layout.addWidget(self.web_view)

        # Ambil IP dan Port dari config (yang seharusnya 127.0.0.1:5000)
        backend_cfg = self.config.get("backend_connection", {})
        host = backend_cfg.get("ip_address", "127.0.0.1")
        port = backend_cfg.get("port", 5000)

        # [FIX KRITIS FINAL] Load the HTML via Flask URL (Same-Origin)
        # Ini adalah satu-satunya cara untuk mengatasi blokir WebEngine/file://
        flask_url = QUrl(f"http://{host}:{port}/map_page")

        self.web_view.load(flask_url)
        print(f"[MapView] Memuat peta dari URL Flask: {flask_url.toString()}")

    def update_data(self, data):
        """
        Kirim update posisi ke JavaScript di dalam WebView.
        """
        lat = data.get("lat", data.get("latitude"))
        lon = data.get("lon", data.get("longitude"))
        hdg = data.get("hdg", data.get("heading", 0))

        if lat is not None and lon is not None:
            script = f"if (typeof updateBoatPosition === 'function') {{ updateBoatPosition({lat}, {lon}, {hdg}); }}"
            self.web_view.page().runJavaScript(script)

        wps = data.get("wps", data.get("waypoints"))
        if wps:
            wps_json = json.dumps(wps)
            script_wp = f"if (typeof updateWaypoints === 'function') {{ updateWaypoints({wps_json}); }}"
            self.web_view.page().runJavaScript(script_wp)

    def update_waypoints(self, waypoints):
        """Slot khusus jika dipanggil langsung dari WaypointsPanel"""
        if waypoints:
            wps_json = json.dumps(waypoints)
            script_wp = f"if (typeof updateWaypoints === 'function') {{ updateWaypoints({wps_json}); }}"
            self.web_view.page().runJavaScript(script_wp)
