# gui/components/map_view.py
# --- MODIFIKASI: Menerima objek 'config' ---

from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QColor, QPen, QBrush, QFont, QPolygonF
from PySide6.QtCore import Slot, QPointF, QRectF, Qt


class MapView(QWidget):
    # --- 1. UBAH TANDA TANGAN FUNGSI __init__ ---
    def __init__(self, config):
        super().__init__()

        # --- 2. SIMPAN OBJEK KONFIGURASI ---
        self.config = config

        self.asv_lat = -6.9175
        self.asv_lon = 107.6191
        self.asv_heading = 0.0  # Simpan heading di sini

        self.waypoints = []
        self.path_history = []

        self.lat_min, self.lat_max = -6.9185, -6.9165
        self.lon_min, self.lon_max = 107.6181, 107.6201

    @Slot(list)
    def update_waypoints(self, new_waypoints):
        self.waypoints = new_waypoints
        self.update()

    @Slot(dict)
    def update_data(self, data):
        """Mengupdate posisi dan heading kapal."""
        self.asv_lat = data.get("latitude", self.asv_lat)
        self.asv_lon = data.get("longitude", self.asv_lon)
        self.asv_heading = data.get("heading", self.asv_heading)

        new_pos = QPointF(self.asv_lon, self.asv_lat)
        if not self.path_history or self.path_history[-1] != new_pos:
            self.path_history.append(new_pos)
            if len(self.path_history) > 200:
                self.path_history.pop(0)
        self.update()

    def _convert_gps_to_pixel(self, lat, lon):
        widget_width = self.width()
        widget_height = self.height()
        if self.lon_max == self.lon_min or self.lat_max == self.lat_min:
            return QPointF(0, 0)
        x = (lon - self.lon_min) / (self.lon_max - self.lon_min) * widget_width
        y = (self.lat_max - lat) / (self.lat_max - self.lat_min) * widget_height
        return QPointF(x, y)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        # (Kode gambar background, grid, jejak, dan waypoint tidak berubah)
        painter.fillRect(self.rect(), QColor("#3498db"))
        grid_pen = QPen(QColor("white"), 1)
        label_font = QFont("Arial", 11, QFont.Bold)
        label_pen = QPen(QColor("white"))
        painter.setFont(label_font)
        num_cols, num_rows = 5, 5
        col_step = self.width() / num_cols
        row_step = self.height() / num_rows
        for i in range(1, num_cols + 1):
            x = i * col_step
            if i < num_cols:
                painter.setPen(grid_pen)
                painter.drawLine(int(x), 0, int(x), self.height())
            painter.setPen(label_pen)
            label_bawah = chr(ord("A") + i - 1)
            label_atas = chr(ord("E") - i + 1)
            painter.drawText(
                QRectF(x - col_step, self.height() - 30, col_step, 30),
                Qt.AlignCenter,
                label_bawah,
            )
            painter.drawText(
                QRectF(x - col_step, 0, col_step, 30), Qt.AlignCenter, label_atas
            )
        for i in range(1, num_rows + 1):
            y = i * row_step
            if i < num_rows:
                painter.setPen(grid_pen)
                painter.drawLine(0, int(y), self.width(), int(y))
            painter.setPen(label_pen)
            label = str(i)
            painter.drawText(
                QRectF(-30, self.height() - y, 30, row_step), Qt.AlignCenter, label
            )
        if len(self.path_history) > 1:
            path_pen = QPen(QColor("#ecf0f1"), 1.5)
            painter.setPen(path_pen)
            pixel_path = [
                self._convert_gps_to_pixel(p.y(), p.x()) for p in self.path_history
            ]
            painter.drawPolyline(pixel_path)
        if len(self.waypoints) > 1:
            waypoint_line_pen = QPen(QColor("#f1c40f"), 1.5, Qt.DashLine)
            painter.setPen(waypoint_line_pen)
            waypoint_pixels = [
                self._convert_gps_to_pixel(wp["lat"], wp["lon"])
                for wp in self.waypoints
            ]
            painter.drawPolyline(waypoint_pixels)
        waypoint_pen = QPen(QColor("#f1c40f"), 2)
        waypoint_brush = QBrush(QColor("#f1c40f"))
        waypoint_font = QFont("Monospace", 9, QFont.Bold)
        for i, wp in enumerate(self.waypoints):
            painter.setPen(waypoint_pen)
            painter.setBrush(waypoint_brush)
            pixel_pos = self._convert_gps_to_pixel(wp["lat"], wp["lon"])
            painter.drawEllipse(pixel_pos, 5, 5)
            painter.setFont(waypoint_font)
            painter.setPen(QColor("white"))
            painter.drawText(pixel_pos + QPointF(8, 5), str(i + 1))

        # --- PERUBAHAN UTAMA: Gambar kapal sebagai panah yang berputar ---
        asv_pixel_pos = self._convert_gps_to_pixel(self.asv_lat, self.asv_lon)

        painter.save()  # Simpan state painter
        painter.translate(asv_pixel_pos)  # Pindahkan titik 0,0 ke posisi kapal
        # Rotasi berdasarkan heading (0 derajat di utara/atas)
        painter.rotate(self.asv_heading)

        painter.setPen(QPen(QColor("black"), 1))
        painter.setBrush(QBrush(QColor("#e74c3c")))

        # Definisikan bentuk panah (segitiga)
        # Puncak panah menunjuk ke atas (arah Y negatif)
        ship_polygon = QPolygonF(
            [
                QPointF(0, -10),  # Hidung kapal
                QPointF(6, 5),  # Ekor kanan
                QPointF(-6, 5),  # Ekor kiri
            ]
        )

        painter.drawPolygon(ship_polygon)
        painter.restore()  # Kembalikan state painter
