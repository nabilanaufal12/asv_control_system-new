# src/navantara_backend/api/endpoints.py
from flask import Blueprint, current_app, Response
import cv2
import eventlet
import traceback  # Impor traceback untuk log error yang lebih detail

from navantara_backend.extensions import socketio

# --- TAMBAHAN IMPORT (Meskipun tidak dipakai langsung di generator) ---
# Ini diperlukan agar Python tahu tipe 'vision_service' saat type hinting,
# tapi akses sebenarnya tetap melalui argumen 'vision_service'.
from navantara_backend.services.vision_service import VisionService

# --- AKHIR TAMBAHAN ---

api_blueprint = Blueprint("api", __name__)


# --- Generator untuk CAM 1 (Hasil AI, Kualitas Disesuaikan) ---
def generate_video_frames_cam1(
    vision_service: VisionService,
):  # Tambahkan type hint (opsional)
    """Generator untuk streaming video frame CAM 1 (hasil AI)."""
    if not vision_service:
        print("ERROR CAM1: Vision service tidak diteruskan!")
        return

    # --- PENGATURAN KUALITAS CAM 1 (SESUAIKAN) ---
    TARGET_WIDTH = 480
    TARGET_HEIGHT = 320
    WEBP_QUALITY = 30
    FRAME_DELAY = 0.08
    # ---------------------------------------------

    while True:
        original_frame = None
        frame_to_encode = None
        try:
            # 1. Ambil frame CAM 1 (hasil AI)
            # --- PERBAIKAN: Gunakan instance 'vision_service' ---
            with vision_service._frame_lock_cam1:
                if vision_service._latest_processed_frame_cam1 is not None:
                    original_frame = vision_service._latest_processed_frame_cam1.copy()
                # --- AKHIR PERBAIKAN ---
                else:
                    eventlet.sleep(FRAME_DELAY)
                    continue

            # 2. Resize frame
            if original_frame is not None:
                resized_frame = cv2.resize(
                    original_frame, (TARGET_WIDTH, TARGET_HEIGHT)
                )
                frame_to_encode = resized_frame
            else:
                eventlet.sleep(FRAME_DELAY)
                continue

            # 3. Encode ke WebP
            if frame_to_encode is not None:
                (flag, encodedImage) = cv2.imencode(
                    ".webp", frame_to_encode, [cv2.IMWRITE_WEBP_QUALITY, WEBP_QUALITY]
                )
                if not flag:
                    print("[Stream CAM1] Gagal encode frame.")
                    eventlet.sleep(FRAME_DELAY)
                    continue

                # 4. Yield frame
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/webp\r\n\r\n"
                    + bytearray(encodedImage)
                    + b"\r\n"
                )

            # 5. Jeda
            eventlet.sleep(FRAME_DELAY)

        except Exception as e:
            print(f"[Stream CAM1] Error dalam generator: {e}")
            traceback.print_exc()
            eventlet.sleep(1)  # Jeda lebih lama jika error


# --- Generator untuk CAM 2 (Mentah, Kualitas Disesuaikan) ---
def generate_video_frames_cam2(
    vision_service: VisionService,
):  # Tambahkan type hint (opsional)
    """Generator untuk streaming video frame CAM 2 (mentah)."""
    if not vision_service:
        print("ERROR CAM2: Vision service tidak diteruskan!")
        return

    # --- PENGATURAN KUALITAS CAM 2 (BISA BERBEDA) ---
    TARGET_WIDTH_CAM2 = 320
    TARGET_HEIGHT_CAM2 = 240
    WEBP_QUALITY_CAM2 = 25
    FRAME_DELAY_CAM2 = 0.05
    # ------------------------------------------------

    while True:
        original_frame = None
        frame_to_encode = None
        try:
            # 1. Ambil frame CAM 2 (mentah)
            # --- PERBAIKAN: Gunakan instance 'vision_service' ---
            with vision_service._frame_lock_cam2:
                if vision_service._latest_raw_frame_cam2 is not None:
                    original_frame = vision_service._latest_raw_frame_cam2.copy()
                # --- AKHIR PERBAIKAN ---
                else:
                    eventlet.sleep(FRAME_DELAY_CAM2)
                    continue

            # 2. Resize frame
            if original_frame is not None:
                resized_frame = cv2.resize(
                    original_frame, (TARGET_WIDTH_CAM2, TARGET_HEIGHT_CAM2)
                )
                frame_to_encode = resized_frame
            else:
                eventlet.sleep(FRAME_DELAY_CAM2)
                continue

            # 3. Encode ke WebP
            if frame_to_encode is not None:
                (flag, encodedImage) = cv2.imencode(
                    ".webp",
                    frame_to_encode,
                    [cv2.IMWRITE_WEBP_QUALITY, WEBP_QUALITY_CAM2],
                )
                if not flag:
                    print("[Stream CAM2] Gagal encode frame.")
                    eventlet.sleep(FRAME_DELAY_CAM2)
                    continue

                # 4. Yield frame
                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/webp\r\n\r\n"
                    + bytearray(encodedImage)
                    + b"\r\n"
                )

            # 5. Jeda
            eventlet.sleep(FRAME_DELAY_CAM2)

        except Exception as e:
            print(f"[Stream CAM2] Error dalam generator: {e}")
            traceback.print_exc()
            eventlet.sleep(1)


# --- Route untuk CAM 1 ---
@api_blueprint.route("/live_video_feed")
def live_video_feed_cam1():
    vision_service_instance = current_app.vision_service
    return Response(
        generate_video_frames_cam1(vision_service_instance),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


# --- Route BARU untuk CAM 2 ---
@api_blueprint.route("/live_video_feed_cam2")
def live_video_feed_cam2():
    vision_service_instance = current_app.vision_service
    return Response(
        generate_video_frames_cam2(vision_service_instance),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


# --- Event Socket.IO (Tidak berubah) ---
@socketio.on("connect")
def handle_connect():
    print("Klien GUI terhubung. Menunggu permintaan stream...")


@socketio.on("disconnect")
def handle_disconnect():
    print("Klien GUI terputus. Menghentikan semua stream data ke klien ini.")
    try:
        if current_app:
            current_app.vision_service.set_gui_listening(False)
            current_app.asv_handler.set_streaming_status(False)
    except RuntimeError:
        print("[API] Peringatan: Tidak bisa akses current_app saat disconnect.")


@socketio.on("request_stream")
def handle_request_stream(json_data):
    status = json_data.get("status", True)
    print(
        f"Menerima permintaan stream dari GUI. Mengatur status streaming ke: {status}"
    )
    try:
        current_app.vision_service.set_gui_listening(status)
        current_app.asv_handler.set_streaming_status(status)
    except Exception as e:
        print(f"[API] Error saat set streaming status: {e}")


@socketio.on("command")
def handle_socket_command(json_data):
    command = json_data.get("command")
    payload = json_data.get("payload", {})
    print(f"Menerima perintah via WebSocket: {command} dengan payload: {payload}")
    vision_commands = ["SET_MODE", "SET_INVERSION"]
    try:
        if command in vision_commands:
            method_name = command.lower()
            if hasattr(current_app.vision_service, method_name):
                getattr(current_app.vision_service, method_name)(payload)
            else:
                print(f"[API] Perintah vision tidak dikenal: {command}")
        else:
            current_app.asv_handler.process_command(command, payload)
    except Exception as e:
        print(f"[API] Error saat memproses command '{command}': {e}")
        traceback.print_exc()
