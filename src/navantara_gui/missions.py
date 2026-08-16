# src/navantara_gui/missions.py
# File ini berisi data untuk misi-misi yang telah ditentukan sebelumnya.

def get_lintasan_a():
    """
    Memuat konfigurasi untuk Lintasan A.
    Berisi 18 waypoint simulasi untuk uji coba fitur portrait.
    """
    waypoints_a = []
    # Membuat 18 titik koordinat berdekatan untuk simulasi
    base_lat = -6.917500
    base_lon = 107.619100
    
    for i in range(18):
        lat = base_lat + (i * 0.000050)
        lon = base_lon + (i * 0.000050)
        waypoints_a.append({"lat": round(lat, 6), "lon": round(lon, 6)})
        
    return {"arena": "A", "waypoints": waypoints_a}

def get_lintasan_b():
    """
    Memuat konfigurasi untuk Lintasan B.
    Mengirim ID arena dan list waypoint kosong agar GUI Web memuat peta.
    """
    return {"arena": "B", "waypoints": []}
