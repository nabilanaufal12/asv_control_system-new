import json
import time
import threading
from flask import Flask, Response, stream_with_context
from flask_cors import CORS

# --- Konfigurasi Dummy ---
DUMMY_MISSION_WPS = [
    {"lat": -6.91750, "lon": 107.61850},  # WP 0
    {"lat": -6.91700, "lon": 107.61900},  # WP 1
    {"lat": -6.91750, "lon": 107.61950},  # WP 2
]

# State global yang akan di-update oleh thread simulator
# dan dibaca oleh thread server
current_telemetry = {
    "latitude": DUMMY_MISSION_WPS[0]["lat"] - 0.0001,  # Mulai sedikit di belakang WP 0
    "longitude": DUMMY_MISSION_WPS[0]["lon"],
    "heading": 45.0,
    "speed": 1.5,
    "nav_target_wp_index": 0,
    "waypoints": DUMMY_MISSION_WPS,
    "active_arena": "A",  # Pastikan 'Arena_A.png' terlihat
    "status": "WAYPOINT NAVIGATION (DUMMY)",
    # Tambahkan field lain jika 'main_local.js' membutuhkannya
    "nav_heading_error": 0,
    "use_dummy_counter": False,
    "debug_waypoint_counter": 0,
}
state_lock = threading.Lock()
# -------------------------

app = Flask(__name__)
# Izinkan koneksi dari browser Anda
CORS(app)


def simulate_mission():
    """
    Thread terpisah yang berjalan di latar belakang,
    memperbarui state telemetri seolah-olah kapal sedang bergerak.
    """

    # Beri waktu 5 detik bagi browser untuk terhubung sebelum memulai
    print("[Simulator] Menunggu 5 detik sebelum memulai misi...")
    time.sleep(5)
    print("[Simulator] Memulai misi dummy...")

    while True:
        # Loop melalui setiap waypoint
        for target_index in range(len(DUMMY_MISSION_WPS)):
            target_wp = DUMMY_MISSION_WPS[target_index]

            print(
                f"[Simulator] Menuju ke WP {target_index} ({target_wp['lat']}, {target_wp['lon']})"
            )

            with state_lock:
                current_telemetry["nav_target_wp_index"] = target_index
                current_telemetry["status"] = (
                    f"WAYPOINT NAVIGATION ({target_index+1}/{len(DUMMY_MISSION_WPS)})"
                )

            # Simulasikan 5 langkah gerakan menuju waypoint
            for step in range(5):
                time.sleep(1)  # Tunggu 1 detik
                with state_lock:
                    # Gerakkan kapal secara linear menuju target (simulasi sederhana)
                    current_lat = current_telemetry["latitude"]
                    current_lon = current_telemetry["longitude"]

                    new_lat = (
                        current_lat + (target_wp["lat"] - current_lat) * 0.2
                    )  # 1/5 jarak
                    new_lon = (
                        current_lon + (target_wp["lon"] - current_lon) * 0.2
                    )  # 1/5 jarak

                    current_telemetry["latitude"] = new_lat
                    current_telemetry["longitude"] = new_lon
                    current_telemetry["heading"] = (
                        current_telemetry["heading"] + 5
                    ) % 360
                    print(f"[Simulator] Update Posisi: {new_lat:.6f}, {new_lon:.6f}")

            # Setelah 5 langkah, anggap kita sudah sampai
            print(f"[Simulator] Mencapai WP {target_index}")
            with state_lock:
                current_telemetry["latitude"] = target_wp["lat"]
                current_telemetry["longitude"] = target_wp["lon"]

        # Misi selesai, ulangi dari awal
        print("[Simulator] Misi selesai. Mengulang...")
        with state_lock:
            # Reset ke posisi awal
            current_telemetry["latitude"] = DUMMY_MISSION_WPS[0]["lat"] - 0.0001
            current_telemetry["longitude"] = DUMMY_MISSION_WPS[0]["lon"]


@app.route("/stream-telemetry")
def stream_telemetry():
    """Endpoint SSE yang mengirimkan data telemetri."""

    def generate_telemetry():
        print("[Server] Klien terhubung ke stream SSE.")
        try:
            while True:
                # Kirim state saat ini
                with state_lock:
                    json_data = json.dumps(current_telemetry)

                # Format SSE: "data: {json_string}\n\n"
                yield f"data: {json_data}\n\n"

                # Kirim update setiap 500ms (0.5 detik)
                time.sleep(0.5)
        except GeneratorExit:
            # Ini terjadi jika browser menutup koneksi
            print("[Server] Klien SSE terputus.")

    # 'text/event-stream' adalah tipe MIME untuk SSE
    return Response(
        stream_with_context(generate_telemetry()), mimetype="text/event-stream"
    )


@app.route("/")
def index():
    return "Dummy Server ASV sedang berjalan. Hubungkan ke /stream-telemetry."


if __name__ == "__main__":
    print("[Server] Memulai simulator thread...")
    simulator_thread = threading.Thread(target=simulate_mission, daemon=True)
    simulator_thread.start()

    print("[Server] Menjalankan server Flask di http://0.0.0.0:5000")
    # 'threaded=True' penting agar server bisa menangani SSE dan simulator
    app.run(host="0.0.0.0", port=5000, threaded=True)
