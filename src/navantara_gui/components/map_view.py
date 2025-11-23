# src/navantara_gui/components/map_view.py
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtWebEngineWidgets import QWebEngineView
import PyQt5.QtCore as QtCore  # [FIX] Pindahkan import ke atas untuk fix E402
import os
import json


class MapView(QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.web_view = QWebEngineView()
        self.layout.addWidget(self.web_view)

        # Load HTML Map
        curr_dir = os.path.dirname(os.path.abspath(__file__))
        html_path = os.path.join(curr_dir, "../../../src/navantara_web/index.html")

        if os.path.exists(html_path):
            self.web_view.load(QtCore.QUrl.fromLocalFile(html_path))
        else:
            self.web_view.setHtml("<h3>Map File Not Found</h3>")

    def update_telemetry(self, data):
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
