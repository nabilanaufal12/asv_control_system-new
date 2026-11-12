# Panduan Migrasi Arsitektur ROS2 & Prompt Antigravity AI — ASV Navantara

Dokumen ini berisi instruksi terstruktur (prompt) untuk diberikan kepada AI Agent (Antigravity AI) guna melakukan *cloning*, audit kode, dan restrukturisasi repositori sistem kontrol ASV Navantara menuju arsitektur modular berbasis **ROS2**, **Nav2**, **TensorRT**, dan **Gazebo**.

---

## 1. Prompt untuk Antigravity AI

Salin blok teks di bawah ini dan berikan langsung kepada Antigravity AI:

```text
Role: Senior Autonomous Marine Robotics & ROS2 Architect.

Task: Clone the repository https://github.com/nabilanaufal12/asv_control_system-new.git (specifically targeting the `asv_control_system-new-jetson-development` directory), audit the current monolithic Python/Arduino codebase, and execute a phased migration plan to a modular ROS2 + Nav2 + TensorRT architecture for our Autonomous Surface Vessel (ASV).

### Step 1 - Repository Clone & Workspace Audit
Execute `git clone https://github.com/nabilanaufal12/asv_control_system-new.git` and inspect the existing project structure inside the `asv_control_system-new-jetson-development` folder:
1. High-level orchestration and REST/GUI entrypoints:
   - `run_backend_headless.py`
   - `run_gui.py`
   - `src/navantara_backend/main.py`
   - `src/navantara_backend/api/endpoints.py`
   - `src/navantara_backend/core/asv_handler.py`
2. State estimation and path logic:
   - `src/navantara_backend/core/navigation.py`
   - `src/navantara_backend/core/kalman_filter.py`
   - `src/navantara_backend/core/mission_logger.py`
3. Perception and hardware services:
   - `src/navantara_backend/services/vision_service.py`
   - `export_tensorrt.py`
   - `src/navantara_backend/vision/best.engine.bak_20260804_142558`
   - `src/navantara_backend/services/serial_service.py`
4. Low-level microcontroller code and configuration:
   - `firmware-asv/firmware/firmware.ino`
   - `firmware-asv/firmware_gps_rtk_gga_f9p/firmware_gps_rtk_gga_f9p.ino`
   - `config/config.json`

### Step 2 - Target Architecture Refactoring Plan
Design and generate a new ROS2 Humble/Iron workspace structure (`navantara_ros2_ws/src/`) that migrates our existing modules into decoupled ROS2 packages while preserving our hardware advantages:

1. Package `navantara_perception`:
   - Wrap the TensorRT inference logic from `vision_service.py` and `export_tensorrt.py` into a dedicated ROS2 lifecycle node.
   - Convert 2D bounding boxes of buoys into 3D/2D relative bearing and distance estimates (`sensor_msgs/msg/PointCloud2` or custom `ObstacleArray` messages) for local costmap injection.

2. Package `navantara_localization`:
   - Migrate `kalman_filter.py` and RTK GNSS NMEA/GGA parsing from `firmware_gps_rtk_gga_f9p.ino` into an Extended Kalman Filter (EKF) node compatible with `robot_localization` (`nav_msgs/msg/Odometry` and `sensor_msgs/msg/NavSatFix`).

3. Package `navantara_navigation`:
   - Replace reactive steering in `navigation.py` with a Nav2 configuration.
   - Set up a rolling `costmap_2d` layer so detected buoys persist in spatial memory even when temporarily lost from camera frames due to wave pitch/roll.

4. Package `navantara_control_bridge`:
   - Refactor `serial_service.py` and `firmware.ino` to implement a deterministic binary protocol (or micro-ROS) with hardware-timed closed-loop PID for heading and thruster ESC control, including a hardware watchdog failsafe.

5. Package `navantara_webgui_bridge`:
   - Isolate `endpoints.py` and `mission_logger.py` into a non-blocking telemetry node using `rosbridge_suite` or an asynchronous FastAPI/Flask subscriber so GUI polling never blocks navigation or control threads.

6. Package `navantara_gazebo_sim`:
   - Create an ASV URDF/SDF hull model with simulated differential thrusters, IMU, GPS, and camera plugins to test `config.json` parameters in a virtual buoy course.

### Step 3 - Immediate Deliverables Required
1. Perform the repository clone and output a dependency & bottleneck audit of `asv_handler.py`, `navigation.py`, `vision_service.py`, and `serial_service.py`.
2. Generate the complete directory tree for the new ROS2 workspace alongside a migration mapping table from the old files to the new ROS2 nodes.
3. Write the initial boilerplate code for the ROS2 `vision_node.py` (wrapping TensorRT) and `serial_bridge_node.py` without breaking existing functionality in the main branch.
```

---

## 2. Pemetaan Migrasi Modul (Lama vs. Baru)

| Modul Lama (`navantara_backend`) | Paket ROS2 Baru | Peran & Peningkatan Utama |
| :--- | :--- | :--- |
| `vision_service.py` & `export_tensorrt.py` | `navantara_perception` | Isolasi inferensi GPU TensorRT (`best.engine`) ke *node* khusus dan proyeksi piksel 2D ke koordinat rintangan spasial. |
| `kalman_filter.py` & `firmware_gps_rtk_gga_f9p.ino` | `navantara_localization` | Fusi sensor IMU dan RTK GPS standar industri menggunakan `robot_localization` (`ekf_node`). |
| `navigation.py` | `navantara_navigation` | Perencanaan jalur berbasis *local costmap* (Nav2) agar kapal tetap mengingat posisi bola pelampung saat terhalang ombak. |
| `serial_service.py` & `firmware.ino` | `navantara_control_bridge` | Eksekusi PID *heading* secara *real-time* di mikrokontroler dengan komunikasi serial deterministik dan *watchdog timer*. |
| `asv_handler.py`, `endpoints.py`, & `mission_logger.py` | `navantara_webgui_bridge` | Pemisahan *thread* API/GUI dan pencatatan log misi agar tidak menghambat siklus kendali kemudi akibat *Python GIL*. |
| `config/config.json` | `navantara_gazebo_sim` & ROS2 *Params* | Pengujian parameter navigasi dan PID di simulator air 3D sebelum diterapkan di kapal fisik. |

---

## 3. Peta Jalan (Roadmap) Pengembangan Kedepan

### Fase 1: Isolasi Proses & Penghapusan Bottleneck (Minggu 1–2)
* Memecah proses `vision_service.py`, `navigation.py`, dan `endpoints.py` yang saat ini menyatu di `asv_handler.py` menjadi *node* ROS2 terpisah.
* Memastikan *frame drop* pada kamera atau *request* HTTP dari dasbor GUI tidak pernah memengaruhi frekuensi pengiriman perintah kontrol ke motor.

### Fase 2: Upgrade Firmware & PID Deterministik (Minggu 3–4)
* Memperbarui `firmware.ino` agar mikrokontroler menjalankan *closed-loop* PID untuk *heading* secara mandiri menggunakan *timer* perangkat keras.
* Menyederhanakan tugas Jetson agar cukup mengirimkan target sudut (*target yaw*) dan kecepatan linier melalui protokol biner yang ringan di `serial_service.py`.

### Fase 3: Implementasi Costmap Spasial & Proyeksi Kamera (Minggu 5–6)
* Mengalibrasi kamera menggunakan skrip di folder `scripts/` untuk memetakan posisi piksel pelampung menjadi jarak dan sudut meter relatif terhadap haluan kapal.
* Memasukkan titik koordinat pelampung ke dalam *rolling local costmap* agar kapal memiliki memori spasial rintangan meskipun objek sempat keluar dari tangkapan layar kamera.

### Fase 4: Simulasi Digital (Gazebo) & Replay Data Log (Minggu 7–8)
* Memanfaatkan rekaman data dari `mission_logger.py` (atau `ros2 bag`) untuk merekonstruksi skenario lintasan di Gazebo.
* Melakukan *tuning* parameter batas kecepatan, radius belok, dan konstanta PID secara virtual sebelum pengujian di air terbuka.