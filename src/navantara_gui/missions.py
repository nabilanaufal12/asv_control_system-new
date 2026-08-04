# src/navantara_gui/missions.py
# File ini berisi data untuk misi-misi yang telah ditentukan sebelumnya.


def get_lintasan_a():
    """
    Memuat konfigurasi untuk Lintasan A.
    Mengirim ID arena dan list waypoint kosong agar GUI Web memuat peta.
    """
    return {"arena": "A", "waypoints": []}


def get_lintasan_b():
    """
    Memuat konfigurasi untuk Lintasan B.
    Mengirim ID arena dan list waypoint kosong agar GUI Web memuat peta.
    """
    return {"arena": "B", "waypoints": []}
