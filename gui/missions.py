# gui/missions.py
# File ini berisi data untuk misi-misi yang telah ditentukan sebelumnya.

# Berdasarkan MapView, area lomba kita berada di sekitar:
# Latitude: -6.9185 hingga -6.9165
# Longitude: 107.6181 hingga 107.6201

# Titik tengah untuk mirroring (garis vertikal di tengah peta)
CENTER_LONGITUDE = 107.6191

# --- Definisi Waypoint untuk Lintasan A ---
# Ini adalah satu-satunya set waypoint yang perlu kita definisikan secara manual.
# Koordinat ini adalah estimasi berdasarkan bentuk lintasan di gambar Anda.
LINTASAN_A_WAYPOINTS = [
    {"lat": -6.91820, "lon": 107.61850},  # 1. Start
    {"lat": -6.91750, "lon": 107.61850},  # 2. Lurus ke atas
    {"lat": -6.91700, "lon": 107.61850},  # 3. Belok
    {"lat": -6.91700, "lon": 107.61910},  # 4. Mendatar
    {"lat": -6.91725, "lon": 107.61950},  # 5. Serong
    {"lat": -6.91775, "lon": 107.61950},  # 6. Lurus ke bawah
    {"lat": -6.91820, "lon": 107.61910},  # 7. Kembali
]


def get_lintasan_a():
    """Mengembalikan salinan dari waypoint Lintasan A."""
    return list(LINTASAN_A_WAYPOINTS)


def get_lintasan_b():
    """
    Menghasilkan waypoint untuk Lintasan B dengan mencerminkan Lintasan A.
    """
    lintasan_b = []
    for wp in LINTASAN_A_WAYPOINTS:
        # Latitude tetap sama
        lat = wp["lat"]

        # Longitude dicerminkan terhadap titik tengah
        old_lon = wp["lon"]
        new_lon = (2 * CENTER_LONGITUDE) - old_lon

        lintasan_b.append({"lat": lat, "lon": new_lon})

    return lintasan_b
