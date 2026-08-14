#!/usr/bin/env python3
"""
export_tensorrt.py
==================
Script untuk mengekspor model YOLOv11 (.pt) ke TensorRT (.engine)
yang dioptimalkan untuk NVIDIA Jetson Orin.

Cara pakai:
    python3 export_tensorrt.py

Konfigurasi bisa diubah di bagian CONFIG di bawah.
"""

import sys
import shutil
from pathlib import Path
from datetime import datetime

# ==============================================================================
#  KONFIGURASI — Ubah sesuai kebutuhan
# ==============================================================================
CONFIG = {
    # File model input (.pt)
    "model_path": "src/navantara_backend/vision/best.pt",
    # Ukuran input model (640 = standar YOLOv11)
    "imgsz": 640,
    # Half precision FP16 (True = lebih cepat di Jetson, WAJIB untuk TensorRT optimal)
    "half": True,
    # Batch size (1 = cocok untuk inferensi real-time)
    "batch": 1,
    # Workspace TensorRT dalam GB (Jetson Orin: 4-8 GB aman)
    "workspace": 4,
    # Output folder
    "output_dir": "src/navantara_backend/vision",
}
# ==============================================================================


def check_prerequisites():
    print("=" * 60)
    print("  Navantara — YOLOv11 -> TensorRT Export Script")
    print("=" * 60)

    try:
        import torch

        cuda_ok = torch.cuda.is_available()
        print(f"\n[CHECK] PyTorch   : {torch.__version__}")
        print(
            f"[CHECK] CUDA      : {'OK - ' + torch.cuda.get_device_name(0) if cuda_ok else 'TIDAK TERSEDIA'}"
        )
        if not cuda_ok:
            print(
                "\n[ERROR] CUDA tidak tersedia! Export TensorRT membutuhkan GPU CUDA."
            )
            sys.exit(1)
    except ImportError:
        print("[ERROR] PyTorch tidak terinstal.")
        sys.exit(1)

    try:
        import ultralytics

        print(f"[CHECK] Ultralytics: {ultralytics.__version__}")
    except ImportError:
        print("[ERROR] Ultralytics tidak terinstal. Jalankan: pip install ultralytics")
        sys.exit(1)

    model_path = Path(CONFIG["model_path"])
    if not model_path.exists():
        print(f"\n[ERROR] File model tidak ditemukan: {model_path}")
        sys.exit(1)

    size_mb = model_path.stat().st_size / (1024 * 1024)
    print(f"[CHECK] Model     : {model_path} ({size_mb:.1f} MB)")


def inspect_model():
    from ultralytics import YOLO

    print("\n" + "=" * 60)
    print("  Inspeksi Model")
    print("=" * 60)
    model = YOLO(CONFIG["model_path"])
    print(f"\n[INFO] Task         : {model.task}")
    print(f"[INFO] Jumlah Kelas : {len(model.names)}")
    print("\n[INFO] Daftar Kelas :")
    for idx, name in model.names.items():
        print(f"       [{idx}] {name}")
    return model


def backup_existing_engine(output_dir, model_name):
    engine_file = output_dir / f"{model_name}.engine"
    if engine_file.exists():
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_path = output_dir / f"{model_name}.engine.bak_{timestamp}"
        shutil.copy2(engine_file, backup_path)
        print(f"\n[BACKUP] Engine lama disimpan: {backup_path.name}")


def export_to_tensorrt(model):
    print("\n" + "=" * 60)
    print("  Export ke TensorRT")
    print("=" * 60)
    model_path = Path(CONFIG["model_path"])
    output_dir = Path(CONFIG["output_dir"])
    model_name = model_path.stem

    backup_existing_engine(output_dir, model_name)

    print(f"\n[CONFIG] imgsz     : {CONFIG['imgsz']}")
    print(f"[CONFIG] half (FP16): {CONFIG['half']}")
    print(f"[CONFIG] batch      : {CONFIG['batch']}")
    print(f"[CONFIG] workspace  : {CONFIG['workspace']} GB")
    print("\n[INFO] Memulai export... (bisa memakan 5-15 menit, jangan matikan!)\n")

    try:
        engine_path = model.export(
            format="engine",
            imgsz=CONFIG["imgsz"],
            half=CONFIG["half"],
            batch=CONFIG["batch"],
            workspace=CONFIG["workspace"],
            device=0,
            verbose=True,
        )
        if engine_path and Path(engine_path).exists():
            size_mb = Path(engine_path).stat().st_size / (1024 * 1024)
            print(f"\n{'=' * 60}")
            print("  EXPORT BERHASIL!")
            print(f"{'=' * 60}")
            print(f"\n[OUTPUT] File  : {engine_path}")
            print(f"[OUTPUT] Ukuran: {size_mb:.1f} MB")
            print("\n[NOTE] Backend akan otomatis memilih .engine ini saat restart.")
        else:
            print("\n[WARNING] Cari file .engine di direktori yang sama dengan best.pt")
    except Exception as e:
        print(f"\n[ERROR] Export gagal: {e}")
        import traceback

        traceback.print_exc()
        sys.exit(1)


def tips_dataset():
    print("\n" + "=" * 60)
    print("  Tips Dataset untuk Akurasi YOLOv11 Maksimal")
    print("=" * 60)
    print(
        """
MASALAH: Model salah menebak antar kelas (Blue_Ball vs Blue_Box, dll.)

PENYEBAB UMUM:
  1. Dataset tidak seimbang antar kelas
  2. Kurang variasi kondisi pencahayaan (terik, mendung, pantulan air)
  3. Kurang variasi jarak kamera ke objek
  4. Bounding box annotasi terlalu longgar

SOLUSI DATASET TERBAIK:
  - Min. 300-500 gambar per kelas (total ~2000 untuk 5 kelas)
  - Foto dari berbagai jarak: 0.5m, 1-2m, 3-5m
  - Variasi sudut: depan, samping, sedikit atas/bawah
  - Kondisi nyata: objek di atas air, sinar matahari, bayangan
  - Bounding box presisi, pas menempel di objek
  - Gunakan Roboflow untuk anotasi + augmentasi otomatis

AUGMENTASI YANG DIREKOMENDASIKAN (via Roboflow/YOLO):
  - Flip Horizontal
  - Brightness +-25%
  - Saturation +-25%
  - Hue +-15deg
  - Blur ringan 1-3px
  - Mosaic (built-in YOLO, sangat membantu)

KELAS YANG SERING KONFLIK:
  Blue_Ball vs Blue_Box   -> tambah foto dari samping (bentuk bola vs kotak lebih jelas)
  Green_Ball vs Green_Box -> pastikan anotasi presisi, hindari bounding box longgar
    """
    )


if __name__ == "__main__":
    check_prerequisites()
    tips_dataset()
    model = inspect_model()

    print("\n" + "=" * 60)
    print(f"  Siap export: {CONFIG['model_path']}")
    print(f"  Output     : {CONFIG['output_dir']}")
    print("=" * 60)
    confirm = input("\nLanjutkan export ke TensorRT? (y/N): ").strip().lower()
    if confirm != "y":
        print("[INFO] Export dibatalkan.")
        sys.exit(0)

    export_to_tensorrt(model)
