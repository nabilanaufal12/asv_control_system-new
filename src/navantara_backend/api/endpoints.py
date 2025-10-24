# src/navantara_backend/api/endpoints.py
from flask import Blueprint, current_app, Response
import cv2
import eventlet

from navantara_backend.extensions import socketio

api_blueprint = Blueprint("api", __name__)

def generate_video_frames(vision_service):
    """
    Generator untuk streaming video frame.
    Menerima instance vision_service untuk menghindari 'RuntimeError'.
    Menyesuaikan kualitas dan resolusi untuk kelancaran.
    """
    if not vision_service:
        print("ERROR: Vision service tidak diteruskan ke generator video!")
        return

    # --- PENGATURAN KUALITAS (SESUAIKAN NILAI INI) ---
    TARGET_WIDTH = 480  # Coba: 640, 480, 320
    TARGET_HEIGHT = 320 # Coba: 480, 320, 240
    WEBP_QUALITY = 30   # Coba: 50, 40, 30, 20 (lebih rendah = kualitas turun, data lebih kecil)
    FRAME_DELAY = 0.08  # Coba: 0.05 (~20 FPS), 0.08 (~12 FPS), 0.1 (~10 FPS) (lebih tinggi = FPS turun)
    # ------------------------------------------------

    while True:
        original_frame = None
        frame_to_encode = None

        # 1. Ambil frame terbaru dengan aman
        with vision_service._frame_lock:
            if vision_service._latest_processed_frame is not None:
                original_frame = vision_service._latest_processed_frame.copy()
            else:
                eventlet.sleep(FRAME_DELAY) # Tunggu jika belum ada frame
                continue

        # 2. Ubah Ukuran Frame (Resize)
        if original_frame is not None:
            try:
                resized_frame = cv2.resize(original_frame, (TARGET_WIDTH, TARGET_HEIGHT))
                frame_to_encode = resized_frame
            except Exception as e:
                 print(f"Error resizing frame: {e}. Menggunakan frame asli.")
                 frame_to_encode = original_frame # Fallback jika resize gagal
        else:
            eventlet.sleep(FRAME_DELAY) # Tunggu jika frame None setelah lock
            continue


        # 3. Encode Frame dengan Kualitas yang Disesuaikan
        if frame_to_encode is not None:
            (flag, encodedImage) = cv2.imencode(
                '.webp', frame_to_encode, [cv2.IMWRITE_WEBP_QUALITY, WEBP_QUALITY]
            )

            if not flag:
                print("Gagal encode frame ke WebP.")
                eventlet.sleep(FRAME_DELAY)
                continue

            # 4. Yield frame dalam format multipart
            yield(b'--frame\r\n' b'Content-Type: image/webp\r\n\r\n' +
                  bytearray(encodedImage) + b'\r\n')

        # 5. Jeda sesuai Frame Rate yang Diinginkan
        eventlet.sleep(FRAME_DELAY)

@api_blueprint.route('/live_video_feed')
def live_video_feed():
    """Rute untuk menyajikan video stream."""
    vision_service_instance = current_app.vision_service
    return Response(generate_video_frames(vision_service_instance),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@socketio.on("connect")
def handle_connect():
    """
    Mencatat koneksi baru dari klien GUI. Server sekarang menunggu
    permintaan eksplisit dari klien untuk memulai streaming data.
    """
    print("Klien GUI terhubung. Menunggu permintaan stream...")


@socketio.on("disconnect")
def handle_disconnect():
    """
    Menangani saat klien terputus, dan memastikan semua stream data berhenti
    untuk menghemat sumber daya.
    """
    print("Klien GUI terputus. Menghentikan semua stream data ke klien ini.")
    # Gunakan try-except untuk handle jika current_app tidak tersedia saat shutdown
    try:
        if current_app:
            current_app.vision_service.set_gui_listening(False)
            current_app.asv_handler.set_streaming_status(False)
    except RuntimeError:
         print("Peringatan: Tidak bisa mengakses current_app saat disconnect (mungkin saat shutdown).")


@socketio.on("request_stream")
def handle_request_stream(json_data):
    """
    Klien meminta server untuk memulai atau menghentikan pengiriman data.
    """
    status = json_data.get("status", True)
    print(
        f"Menerima permintaan stream dari GUI. Mengatur status streaming ke: {status}"
    )
    current_app.vision_service.set_gui_listening(status)
    current_app.asv_handler.set_streaming_status(status)


@socketio.on("command")
def handle_socket_command(json_data):
    """Menerima dan meneruskan perintah umum dari GUI ke layanan yang sesuai."""
    command = json_data.get("command")
    payload = json_data.get("payload", {})
    print(f"Menerima perintah via WebSocket: {command} dengan payload: {payload}")

    vision_commands = ["SET_MODE", "SET_INVERSION"]

    if command in vision_commands:
        method_name = command.lower()
        if hasattr(current_app.vision_service, method_name):
            try:
                getattr(current_app.vision_service, method_name)(payload)
            except Exception as e:
                print(f"[API] Error saat menjalankan perintah vision '{command}': {e}")
        else:
            print(f"[API] Perintah vision tidak dikenal: {command}")
    else:
        # Untuk semua perintah lainnya kirim ke asv_handler
        try:
             current_app.asv_handler.process_command(command, payload)
        except Exception as e:
             print(f"[API] Error saat memproses perintah ASV '{command}': {e}")