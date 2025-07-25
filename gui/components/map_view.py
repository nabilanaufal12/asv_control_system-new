# gui/components/map_view.py
# Komponen untuk menampilkan peta, posisi ASV, dan waypoints.

from PySide6.QtWidgets import QGroupBox, QVBoxLayout, QLabel
from PySide6.QtGui import QPixmap, QImage, QPainter, QColor, QFont, QPen, QBrush
from PySide6.QtCore import Qt, Slot, QPointF
from collections import deque

class MapView(QGroupBox):
    """
    Sebuah GroupBox yang menampilkan peta simulasi dengan grid berlabel,
    posisi ASV, dan jejaknya.
    """
    def __init__(self, title="Map View"):
        super().__init__(title)

        self.map_label = QLabel("Initializing map...")
        self.map_label.setAlignment(Qt.AlignCenter)
        self.map_label.setStyleSheet("background-color: #2E4053;")
        self.map_label.setMinimumSize(640, 480) 
        
        layout = QVBoxLayout()
        layout.addWidget(self.map_label)
        self.setLayout(layout)
        
        self.initial_pos = None
        self.current_pos = QPointF(0, 0)
        self.path_history = deque(maxlen=100)
        self.waypoints = []

        self.scale = 500000 

    @Slot(dict)
    def update_data(self, data):
        """Menerima data telemetri dan mengupdate posisi ASV."""
        lat = data.get("latitude")
        lon = data.get("longitude")

        if lat is None or lon is None:
            return
            
        if self.initial_pos is None:
            self.initial_pos = QPointF(lon, lat)

        self.current_pos.setX((lon - self.initial_pos.x()) * self.scale)
        self.current_pos.setY(-(lat - self.initial_pos.y()) * self.scale) 

        self.path_history.append(self.current_pos)
        self.draw_map()

    @Slot(list)
    def update_waypoints(self, waypoints_list):
        """Menerima daftar waypoints dan menyimpannya untuk digambar."""
        print(f"[MapView] Menerima waypoints baru: {waypoints_list}")
        self.waypoints = waypoints_list
        self.draw_map()

    def draw_map(self):
        """Menggambar semua elemen di peta: grid, jejak, ASV, dan waypoints."""
        width = self.map_label.width()
        height = self.map_label.height()
        if width <= 0 or height <= 0: return
        
        image = QImage(width, height, QImage.Format_RGB32)
        image.fill(QColor("#2E4053"))
        painter = QPainter(image)
        
        # --- PERUBAHAN UTAMA: Gambar Grid Berlabel 5x5 ---
        padding = 50 # Ruang di tepi untuk label
        grid_area_width = width - (2 * padding)
        grid_area_height = height - (2 * padding)
        cell_width = grid_area_width / 5
        cell_height = grid_area_height / 5

        font = QFont(); font.setPointSize(12); font.setBold(True)
        painter.setFont(font)
        painter.setPen(QColor("white"))

        # Gambar garis vertikal dan label A-E
        labels_x = ["A", "B", "C", "D", "E"]
        for i in range(6):
            x = padding + (i * cell_width)
            painter.setPen(QPen(QColor(80, 100, 120), 2)) # Garis grid lebih tebal
            painter.drawLine(int(x), padding, int(x), height - padding)
            if i < 5:
                painter.setPen(QColor("white")) # Warna label
                label_x_pos = padding + (i * cell_width) + (cell_width / 2) - (font.pointSize() / 2)
                painter.drawText(int(label_x_pos), height - padding + 25, labels_x[i])
                painter.drawText(int(label_x_pos), padding - 15, labels_x[i])


        # Gambar garis horizontal dan label 1-5
        for i in range(6):
            y = padding + (i * cell_height)
            painter.setPen(QPen(QColor(80, 100, 120), 2))
            painter.drawLine(padding, int(y), width - padding, int(y))
            if i < 5:
                painter.setPen(QColor("white"))
                label_y_pos = height - padding - (i * cell_height) - (cell_height / 2) + (font.pointSize() / 2)
                painter.drawText(padding - 25, int(label_y_pos), str(i + 1))

        # Gambar jejak (path) - Disesuaikan dengan area grid
        if len(self.path_history) > 1:
            painter.setPen(QPen(QColor("#1ABC9C"), 2))
            # Asumsikan posisi (0,0) adalah tengah grid
            center_x, center_y = width / 2, height / 2
            path_points = [QPointF(p.x() + center_x, p.y() + center_y) for p in self.path_history]
            painter.drawPolyline(path_points)

        # Gambar Waypoints - Disesuaikan dengan area grid
        painter.setPen(QPen(QColor("#F1C40F"), 2))
        painter.setBrush(QBrush(QColor(241, 196, 15, 150)))
        for i, wp in enumerate(self.waypoints):
            if self.initial_pos:
                wp_x = (wp['lon'] - self.initial_pos.x()) * self.scale + (width / 2)
                wp_y = -(wp['lat'] - self.initial_pos.y()) * self.scale + (height / 2)
                painter.drawEllipse(QPointF(wp_x, wp_y), 6, 6)

        # Gambar posisi ASV (lingkaran merah) - Disesuaikan dengan area grid
        asv_draw_pos = QPointF(self.current_pos.x() + (width / 2), self.current_pos.y() + (height / 2))
        painter.setBrush(QBrush(QColor("#E74C3C")))
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(asv_draw_pos, 8, 8)

        painter.end()
        self.map_label.setPixmap(QPixmap.fromImage(image))
