# Catatan Progres Migrasi Arsitektur ROS 2 ASV Navantara

**Tanggal**: 15 November 2026  
**Status**: Fase 1 & 2 (Selesai 100%), Fase 3 (HIL Bringup & Jetson Build Selesai, Menunggu Sambungan Fisik Periferal USB)  
**Branch Git**: [`ros2-migration`](https://github.com/nabilanaufal12/asv_control_system-new/tree/ros2-migration) (Commit `8262090`)  
**Target Hardware**: 
- **Laptop Host / Visualizer**: Laptop Razer Blade 14 (Ubuntu 24.04 LTS x86_64, ROS 2 Jazzy)
- **Edge Companion Computer**: NVIDIA Jetson Orin Nano 8GB (`navantara@192.168.1.20`, Ubuntu 22.04 LTS aarch64, ROS 2 Humble, TensorRT 10.3.0, CUDA 12.6)
- **Actuator Controller**: ESP32 NodeMCU (Dual Thruster PWM & Rudder Servo via UART)
- **High-Precision GNSS**: u-blox ZED-F9P RTK GNSS Module (USB Serial NMEA/GGA/RMC)
- **Vision Sensors**: Logitech C930c USB Surface Camera & XWF 1080P Underwater Camera

---

## 1. Ringkasan Eksekutif & Pencapaian Utama

Sistem kontrol monolitik ASV Navantara berbasis Python multithread dan Arduino lama telah berhasil ditransformasikan ke dalam fondasi arsitektur **modular ROS 2 (Humble & Jazzy Cross-Compatible)**. Seluruh 7 paket ROS 2 di [`navantara_ros2_ws`](file:///home/mizuuu/Documents/Koleksi_KKI_Navantara_ASV/4_Firmware_dan_Source_Code/asv_control_system_jetson/navantara_ros2_ws) terbukti **100% buildable dan lulus verifikasi eksekusi** baik di Laptop pengembang maupun langsung di komputer tepi NVIDIA Jetson Orin Nano.

```
       [Logitech C930c USB]           [u-blox ZED-F9P RTK]           [ESP32 Actuator MCU]
                 │                              │                              │
                 ▼ (/dev/video_surface)         ▼ (/dev/ttyGPS)                ▼ (/dev/ttyASV)
       ┌──────────────────┐           ┌──────────────────┐           ┌──────────────────┐
       │   vision_node    │           │  rtk_gnss_node   │           │serial_bridge_node│
       │ (TensorRT FP16)  │           │(NMEA GGA/RMC RTK)│           │ (50 Hz UART CRC) │
       └─────────┬────────┘           └─────────┬────────┘           └─────────▲────────┘
                 │ /perception/obstacles        │ /fix, /compass/heading       │ /cmd_vel
                 ▼                              ▼                              │
       ┌─────────────────────────────────────────────────────────┐             │
       │                 buoy_navigator_node                     ├─────────────┘
       │ • 4.0s Spatial Memory Buffer (Frame 'odom')             │
       │ • KKI Gate Pairing (Red=Port, Green=Stbd, Blue=Nudge)   │
       │ • Adaptive Pure Pursuit Steering Control Law            │
       │ • 2D Rolling Local Costmap (20m x 20m, 10 Hz)           │
       └────────────────────────────┬────────────────────────────┘
                                    │ /local_costmap/costmap, /navigation/markers
                                    ▼
       ┌─────────────────────────────────────────────────────────┐
       │             RViz2 (Auto-Tracking base_link)             │
       │       Catamaran 3D Hull + Gate Lines + Dynamic Status   │
       └─────────────────────────────────────────────────────────┘
```

---

## 2. Rincian Pencapaian Tiap Fase

### Fase 1 — Fondasi & Standarisasi ROS 2 (100% Selesai)
* **Standardisasi Build System**: Memperbaiki arsitektur Python packaging (`setup.cfg` + `setuptools`) sehingga direktori libexec `install/<pkg>/lib/<pkg>/` terisi sempurna.
* **Resolusi Dependensi**: Menyelesaikan konflik NumPy 2.x dengan OpenCV dan rosidl C generators pada Ubuntu 24.04 Jazzy.
* **QoS Penyelarasan**: Mengkonfigurasi seluruh komunikasi status sensor ke profil `RELIABLE` untuk mencegah peringatan *QoS incompatible*.

### Fase 2 — Simulasi Closed-Loop & Navigasi Otonom (100% Selesai)
* **Model 3D Katamaran di RViz2**: Model URDF twin-hull catamaran lengkap ([`asv.urdf.xacro`](file:///home/mizuuu/Documents/Koleksi_KKI_Navantara_ASV/4_Firmware_dan_Source_Code/asv_control_system_jetson/navantara_ros2_ws/src/navantara_gazebo_sim/urdf/asv.urdf.xacro)) dipublikasikan via `robot_state_publisher` ke `/robot_description`.
* **Navigator Otonom Pelampung KKI ([`buoy_navigator_node.py`](file:///home/mizuuu/Documents/Koleksi_KKI_Navantara_ASV/4_Firmware_dan_Source_Code/asv_control_system_jetson/navantara_ros2_ws/src/navantara_navigation/navantara_navigation/buoy_navigator_node.py))**:
  - Buffer memori spasial 4.0 detik di frame global `odom`.
  - Gate pairing antara bola merah (port) dan hijau (starboard).
  - Mode proteksi pelampung tunggal (*single-buoy offset* $\pm 1.8\text{ m}$).
  - Hukum kendali *Pure Pursuit* adaptif berbasis limit radius kurvatur.
* **2D Rolling Costmap Spasial (`/local_costmap/costmap`)**: Grid okupansi 10 Hz ($20\text{ m} \times 20\text{ m}$, resolusi $0.1\text{ m}$) yang selalu berpusat mengikuti posisi kapal dengan inflasi rintangan bertingkat.
* **Auto-Tracking Kamera RViz2**: Menetapkan `Target Frame: base_link` pada konfigurasi RViz2 sehingga kamera otomatis mengikuti kapal saat berlayar.

### Fase 3 — Hardware-in-the-Loop (HIL) & Edge Bringup (Status Terkini)
* **Verifikasi Model TensorRT Asli Jetson Orin Nano**:
  - Model inferensi native FP16 (`/home/navantara/navantara/src/navantara_backend/vision/best.engine`, 8.3 MB) diuji langsung di GPU Orin Nano.
  - Memetakan 5 kelas deteksi:
    ```python
    {0: 'Blue_Ball', 1: 'Blue_Box', 2: 'Green_Ball', 3: 'Green_Box', 4: 'Red_Ball'}
    ```
* **Penyelarasan Logika Perseptual & Penghindaran Rintangan**:
  - **Red** (`Red_Ball`): Batas kiri gerbang (*Port gate* / $+Y$).
  - **Green** (`Green_Ball`, `Green_Box`): Batas kanan gerbang (*Starboard gate* / $-Y$).
  - **Blue** (`Blue_Ball`, `Blue_Box`): Rintangan tengah kanal. Navigator secara otomatis melakukan manuver *lateral avoidance nudge* dan menerbitkan visual marker kubus biru royal `(0.1, 0.3, 1.0)`.
* **Node Driver RTK GNSS NMEA ([`rtk_gnss_node.py`](file:///home/mizuuu/Documents/Koleksi_KKI_Navantara_ASV/4_Firmware_dan_Source_Code/asv_control_system_jetson/navantara_ros2_ws/src/navantara_localization/navantara_localization/rtk_gnss_node.py))**:
  - Dibuat khusus di dalam `navantara_localization`.
  - Parsing `$GNGGA`/`$GPGGA` dan `$GNRMC`/`$GPRMC` dengan validasi checksum XOR.
  - Matriks kovariansi dinamis berbasis status RTK:
    - *RTK Fixed (Fix 4)*: $\sigma = 0.015\text{ m}$ ($\sigma^2 = 2.25 \times 10^{-4}$)
    - *RTK Float (Fix 5)*: $\sigma = 0.20\text{ m}$ ($\sigma^2 = 0.04$)
    - *DGPS (Fix 2)*: $\sigma = 0.80\text{ m}$ ($\sigma^2 = 0.64$)
    - *Standalone GPS (Fix 1)*: $\sigma = 2.50\text{ m}$ ($\sigma^2 = 6.25$)
  - Menerbitkan `sensor_msgs/msg/NavSatFix` (`/fix`) dan `std_msgs/msg/Float32` (`/compass/heading`).
* **Aturan Persistent Udev ([`99-navantara-asv.rules`](file:///home/mizuuu/Documents/Koleksi_KKI_Navantara_ASV/4_Firmware_dan_Source_Code/asv_control_system_jetson/scripts/99-navantara-asv.rules))**:
  - Symlink `/dev/ttyASV` untuk mikrokontroler ESP32 (CP210x, CH340, FTDI, ESP32 USB).
  - Symlink `/dev/ttyGPS` untuk modul u-blox ZED-F9P.
  - Symlink `/dev/video_surface` dan `/dev/video_underwater`.
* **Konfigurasi Master Bringup Headless ([`navantara_bringup.launch.py`](file:///home/mizuuu/Documents/Koleksi_KKI_Navantara_ASV/4_Firmware_dan_Source_Code/asv_control_system_jetson/navantara_ros2_ws/src/navantara_navigation/launch/navantara_bringup.launch.py))**:
  - Saat `use_laptop_mock:=false`, sistem otomatis memuat `vision_node` (TensorRT engine di Jetson GPU), `serial_bridge_node` (`/dev/ttyASV`), `rtk_gnss_node` (`/dev/ttyGPS`), dan `buoy_navigator_node` otonom.
* **Build Sukses di Jetson Orin Nano**:
  - Seluruh 7 paket ROS 2 di-build pada arsitektur aarch64 (ROS 2 Humble) dalam waktu **8.50 detik** tanpa satu pun error.

---

## 3. Matriks Status Paket ROS 2

| Paket | Tipe Build | Host (Jazzy) | Jetson (Humble) | Fitur Utama |
| :--- | :--- | :---: | :---: | :--- |
| **`navantara_interfaces`** | `ament_cmake` | ✅ 100% | ✅ 100% | Pesan kustom: `Obstacle.msg`, `ObstacleArray.msg`, `ASVTelemetry.msg` |
| **`navantara_perception`** | `ament_python` | ✅ 100% | ✅ 100% | Intelejen visual TensorRT 10.3 (`best.engine`), mock generator di laptop, estimasi jarak hibrida, pointcloud2 |
| **`navantara_control_bridge`** | `ament_python` | ✅ 100% | ✅ 100% | Komunikasi serial UART ber-CRC16, hydrodynamics simulator (mock), TF dynamic `odom` $\to$ `base_link`, failsafe watchdog 0.5s |
| **`navantara_localization`** | `ament_python` | ✅ 100% | ✅ 100% | Node driver NMEA u-blox RTK GNSS (`rtk_gnss_node`), konfigurasi EKF `robot_localization` (`ekf.yaml`) |
| **`navantara_navigation`** | `ament_python` | ✅ 100% | ✅ 100% | Navigator otonom pelampung KKI (`buoy_navigator_node`), 2D rolling costmap 10 Hz, master bringup launch file |
| **`navantara_webgui_bridge`** | `ament_python` | ✅ 100% | ✅ 100% | Telemetri CSV logger non-blocking dan bridging HTTP/WebSocket dashboard |
| **`navantara_gazebo_sim`** | `ament_cmake` | ✅ 100% | ✅ 100% | Model URDF 3D katamaran (`asv.urdf.xacro`) dan arena air lintasan lomba KKI |

---

## 4. Langkah Pengujian Lapangan / Bench Test Berikutnya

1. **Penyambungan Fisik Periferal USB ke Jetson Orin Nano**:
   - Sambungkan **Kamera Permukaan (Logitech C930c)** ke port USB 3.0 Jetson.
   - Sambungkan **ESP32 Microcontroller** ke port USB Jetson.
   - Sambungkan **u-blox ZED-F9P RTK GNSS** ke port USB Jetson.
   > [!CAUTION]
   > **Prosedur Keamanan Thruster**: Pastikan baterai utama motor thruster / ESC dalam keadaan **TERPUTUS (OFF)** saat pengujian meja (*bench dry run*) untuk mencegah putaran baling-baling yang tidak disengaja.

2. **Aplikasi Aturan Udev pada Jetson**:
   Jalankan perintah berikut di Jetson (atau via terminal):
   ```bash
   sudo cp /home/navantara/navantara/scripts/99-navantara-asv.rules /etc/udev/rules.d/
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```
   Verifikasi symlink:
   ```bash
   ls -la /dev/ttyASV /dev/ttyGPS /dev/video_surface
   ```

3. **Peluncuran Hardware Bringup Otonom**:
   ```bash
   ros2 launch navantara_navigation navantara_bringup.launch.py use_laptop_mock:=false
   ```

4. **Verifikasi Topik & Metrik Kinerja**:
   - Inferensi TensorRT: `ros2 topic hz /perception/obstacles` (Target: $\ge 25\text{ Hz}$).
   - Telemetri RTK GNSS: `ros2 topic echo /fix --once` (Verifikasi akurasi RTK).
   - Respon Kemudi Otonom: Letakkan objek merah/hijau di depan kamera dan amati publikasi `/cmd_vel`.
