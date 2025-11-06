# src/navantara_gui/missions.py
# File ini berisi data untuk misi-misi yang telah ditentukan sebelumnya.

# Berdasarkan MapView, area lomba kita berada di sekitar:
# Latitude: -6.9185 hingga -6.9165
# Longitude: 107.6181 hingga 107.6201


# --- [REFAKTORISASI TAHAP 1] ---
# Data waypoint yang di-hardcode (LINTASAN_A_WAYPOINTS) dan 
# logika mirroring (CENTER_LONGITUDE) telah dihapus/dikomentari.
# Fungsi sekarang hanya mengembalikan ID arena dan list waypoint kosong
# untuk memicu pemuatan peta di GUI Web.

# Titik tengah untuk mirroring (garis vertikal di tengah peta)
# CENTER_LONGITUDE = 107.6191

# --- Definisi Waypoint untuk Lintasan A ---
# LINTASAN_A_WAYPOINTS = [
#     {"lat": -6.91820, "lon": 107.61850},  # 1. Start
#     {"lat": -6.91750, "lon": 107.61850},  # 2. Lurus ke atas
#     {"lat": -6.91700, "lon": 107.61850},  # 3. Belok
#     {"lat": -6.91700, "lon": 107.61910},  # 4. Mendatar
#     {"lat": -6.91725, "lon": 107.61950},  # 5. Serong
#     {"lat": -6.91775, "lon": 107.61950},  # 6. Lurus ke bawah
#     {"lat": -6.91820, "lon": 107.61910},  # 7. Kembali
# ]
# --- [AKHIR REFAKTORISASI TAHAP 1] ---


def get_lintasan_a():
    """
    Memuat konfigurasi untuk Lintasan A.
    HANYA mengirim ID arena dan list waypoint KOSONG.
    """
    # return list(LINTASAN_A_WAYPOINTS) # <-- Logika lama
    return {
        "arena": "A",
        "waypoints": []
    }


def get_lintasan_b():
    """
    Memuat konfigurasi untuk Lintasan B.
    HANYA mengirim ID arena dan list waypoint KOSONG.
    """
    # Logika mirroring lama dihapus
    # lintasan_b = []
    # for wp in LINTASAN_A_WAYPOINTS:
    #     lat = wp["lat"]
    #     old_lon = wp["lon"]
    #     new_lon = (2 * CENTER_LONGITUDE) - old_lon
    #     lintasan_b.append({"lat": lat, "lon": new_lon})
    # return lintasan_b # <-- Logika lama
    
    return {
        "arena": "B",
        "waypoints": []
    }