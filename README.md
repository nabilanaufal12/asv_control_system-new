# 🚢 Navantara — ASV Ground Control System

> **Autonomous Surface Vehicle (ASV) Ground Control Station** untuk kompetisi **Kontes Kapal Indonesia (KKI) 2026**.  
> Dikembangkan oleh Tim Navantara.

---

## 📋 Deskripsi

Navantara adalah sistem kendali terintegrasi untuk wahana permukaan air otonom (ASV) yang terdiri dari:

- **Backend Server** — Flask + SocketIO untuk komunikasi real-time antara Ground Control Station (GCS) dengan mikrokontroler ESP32 melalui serial.
- **Desktop GUI** — Aplikasi PySide6 untuk operator, menampilkan telemetri, video stream, manajemen waypoint, dan kontrol manual/otomatis.
- **Web Dashboard** — Antarmuka browser untuk monitoring trajectory, telemetri, dan status navigasi secara real-time.
- **Firmware ESP32** — Firmware Arduino untuk kontrol aktuator (servo, motor), pembacaan sensor (GPS, IMU), dan navigasi waypoint onboard.

---

## 🏗️ Arsitektur Sistem

```
┌─────────────────┐     WebSocket/SSE      ┌──────────────────┐
│   Desktop GUI   │◄─────────────────────►  │  Flask Backend   │
│   (PySide6)     │                         │  (SocketIO)      │
└─────────────────┘                         └────────┬─────────┘
                                                     │ Serial (UART)
┌─────────────────┐     HTTP/SSE            ┌────────▼─────────┐
│  Web Dashboard  │◄─────────────────────►  │     ESP32        │
│  (Browser)      │                         │  (Arduino FW)    │
└─────────────────┘                         └──────────────────┘
```

---

## 📁 Struktur Proyek

```
navantara/
├── config/
│   └── config.json                  # Konfigurasi utama (PID, vision, serial, dsb.)
├── firmware-asv/
│   └── firmware/                    # Firmware Arduino untuk ESP32
├── scripts/
│   ├── check_cameras.py             # Diagnosa kamera (OpenCV)
│   ├── check_labels.py              # Cek label model YOLO
│   ├── diagnose_serial.py           # Diagnosa koneksi serial
│   └── test_koneksi.py              # Tes koneksi backend
├── src/
│   ├── navantara_backend/
│   │   ├── api/
│   │   │   └── endpoints.py         # REST & WebSocket API endpoints
│   │   ├── core/
│   │   │   ├── asv_handler.py       # State manager & command processor
│   │   │   ├── kalman_filter.py     # Extended Kalman Filter (EKF)
│   │   │   ├── mission_logger.py    # Logger misi (CSV export)
│   │   │   └── navigation.py        # PID Controller & navigasi
│   │   ├── services/
│   │   │   ├── serial_service.py    # Komunikasi serial ESP32
│   │   │   └── vision_service.py    # AI Vision (YOLOv11 + OpenCV)
│   │   ├── vision/
│   │   │   ├── best.pt              # Model YOLOv11 terlatih
│   │   │   └── overlay_utils.py     # Utilitas overlay visual deteksi
│   │   ├── extensions.py            # Flask-SocketIO instance
│   │   └── main.py                  # App factory & routing
│   ├── navantara_gui/
│   │   ├── assets/
│   │   │   ├── resources/           # QSS theme files (dark/light)
│   │   │   └── *.png                # Logo sponsor & universitas
│   │   ├── components/
│   │   │   ├── control_panel.py     # Panel kontrol mode (AUTO)
│   │   │   ├── dashboard.py         # Dashboard telemetri & status
│   │   │   ├── debug_panel.py       # Panel debug waypoint
│   │   │   ├── header.py            # Header bar aplikasi
│   │   │   ├── settings_panel.py    # Pengaturan AI Vision
│   │   │   ├── video_view.py        # Dual camera viewer
│   │   │   └── waypoints_panel.py   # Manajemen waypoint & misi foto
│   │   ├── views/
│   │   │   └── main_window.py       # Main window layout
│   │   ├── api_client.py            # WebSocket client ke backend
│   │   ├── main_gui.py              # Entry point GUI
│   │   └── missions.py              # Template lintasan Arena A/B
│   └── navantara_web/
│       ├── templates/
│       │   ├── index.html            # Dashboard web utama
│       │   └── debug_telemetry.html  # Halaman debug telemetri
│       └── static/
│           ├── css/                  # Stylesheet
│           ├── js/                   # JavaScript (telemetri, trajectory)
│           ├── images/               # Gambar arena & logo
│           └── lib/                  # Library pihak ketiga (Leaflet, dll.)
├── run_backend_headless.py           # Entry point backend server
├── run_gui.py                        # Entry point desktop GUI
├── requirements.txt                  # Dependensi Python
├── Penting.txt                       # Catatan utilitas
└── .flake8                           # Konfigurasi linter
```

---

## ⚡ Instalasi & Setup

### Prasyarat

- **Python 3.10+**
- **pip** (Python package manager)
- **USB Serial** — Kabel USB ke ESP32 (CH340/CP2102)
- **Webcam** — Minimal 1 kamera USB (opsional: 2 kamera untuk surface + underwater)

### Langkah Instalasi

```bash
# 1. Clone repository
git clone https://github.com/nabilanaufal12/asv_control_system-new.git
cd navantara

# 2. Buat virtual environment
python3 -m venv .venv
source .venv/bin/activate

# 3. Install dependensi
pip install -r requirements.txt
```

---

## 🚀 Cara Menjalankan

### Backend Server (Wajib dijalankan terlebih dahulu)

```bash
python3 run_backend_headless.py
```

Server akan berjalan di `http://0.0.0.0:5000`.

### Desktop GUI (Opsional — untuk operator)

```bash
python3 run_gui.py
```

### Web Dashboard (Opsional — via browser)

Buka browser dan akses:

```
http://localhost:5000/
```

---

## 🎯 Fitur Utama

| Fitur | Deskripsi |
|---|---|
| **Dual Camera Stream** | Streaming video dari 2 kamera (Surface & Underwater) |
| **AI Object Detection** | Deteksi objek real-time menggunakan YOLOv11 (bola, kotak) |
| **Waypoint Navigation** | Manajemen waypoint dengan template Arena A/B |
| **PID Heading Control** | Kontrol heading otomatis dengan PID + Kalman Filter |
| **Live Telemetry** | Monitoring GPS, heading, speed, battery secara real-time |
| **Trajectory Visualization** | Visualisasi lintasan di canvas web interaktif |
| **Photography Mission** | Misi foto otomatis pada segmen waypoint tertentu |
| **Mission Logging** | Pencatatan data misi ke format CSV |
| **Swap Camera** | Tukar tampilan kamera surface ↔ underwater |

---

## ⚙️ Konfigurasi

Semua konfigurasi disimpan di `config/config.json`:

- **`serial_connection`** — Baud rate dan deskriptor USB auto-connect
- **`navigation`** — Parameter PID, jarak waypoint reach, lookahead distance
- **`actuators`** — Batas servo dan PWM motor
- **`vision`** — Threshold deteksi, kelas objek, indeks kamera
- **`camera_detection`** — Focal length dan ukuran objek nyata (untuk estimasi jarak)

---

## 🧹 Utilitas

```bash
# Bersihkan cache Python
find . -type d -name "__pycache__" -exec rm -r {} +

# Format kode (PEP 8)
black .

# Cek kualitas kode
flake8 .

# Diagnosa kamera
python3 scripts/check_cameras.py

# Diagnosa koneksi serial
python3 scripts/diagnose_serial.py
```

---

## 📜 Lisensi

Proyek internal Tim Navantara untuk Kontes Kapal Indonesia (KKI) 2026.

---

<p align="center">
  <b>🚢 Navantara — Mengarungi Lautan dengan Teknologi 🚢</b>
</p>
