# AckermanSFM — Sistem ADAS Berbasis ROS

> **Proyek Akhir Sarjana Terapan** | Teknik Komputer | 2026  
> Pengembang: **Azra Pralingga**

---

## 📋 Deskripsi Proyek

**AckermanSFM** adalah implementasi sistem **Advanced Driver Assistance System (ADAS)** berbasis ROS (Robot Operating System) pada kendaraan bergerak (Ackerman steering). Sistem ini mengintegrasikan beberapa modul ADAS secara bersamaan dalam satu platform:

| Modul | Fungsi |
|---|---|
| **Social Force Model (SFM)** | Pengendalian jarak dan mengikuti objek di depan secara adaptif |
| **Lane Keeping Assist (LKA)** | Menjaga kendaraan tetap di jalur menggunakan deteksi marka jalan |
| **Fuzzy Logic Controller** | Pengontrol akselerasi/deselerasi cerdas berbasis logika fuzzy Sugeno |
| **Object Detection (YOLOv8)** | Deteksi dan segmentasi objek real-time menggunakan kamera |
| **LiDAR Processing** | Pembacaan jarak obstacle menggunakan RPLiDAR |
| **Time Synchronization** | Sinkronisasi timestamp antar sensor |

---

## 🏗️ Arsitektur Sistem

```
┌─────────────────────────────────────────────────────────┐
│                    ROS Ecosystem                         │
│                                                          │
│  ┌───────────┐   ┌──────────────┐   ┌────────────────┐  │
│  │  RPLiDAR  │   │   Kamera     │   │   Encoder      │  │
│  │  (LiDAR)  │   │  (USB/CSI)   │   │   (Odometry)   │  │
│  └─────┬─────┘   └──────┬───────┘   └───────┬────────┘  │
│        │                │                   │            │
│        ▼                ▼                   ▼            │
│  ┌───────────┐   ┌──────────────┐   ┌────────────────┐  │
│  │  lidar_   │   │  yolov8_     │   │  time_sync_    │  │
│  │  scan_    │   │  seg_ros     │   │  pkg           │  │
│  │  node     │   │  (detection) │   │                │  │
│  └─────┬─────┘   └──────┬───────┘   └───────┬────────┘  │
│        │                │                   │            │
│        ▼                ▼                   │            │
│  ┌───────────┐   ┌──────────────┐           │            │
│  │  SFM +    │◄──│  LKA Node    │           │            │
│  │  Fuzzy    │   │  (Lane Error)│           │            │
│  └─────┬─────┘   └──────────────┘           │            │
│        │                                    │            │
│        ▼                                    │            │
│  ┌──────────────────────────────────────────┴──────────┐ │
│  │          Drive Node (Kontrol PWM Motor)              │ │
│  └─────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
```

---

## 📁 Struktur Repository

```
AckermanSFM/
├── ackerman/                    # Package utama ROS
│   ├── src/
│   │   ├── drive.cpp            # Node utama pengemudi kendaraan
│   │   ├── sfm.cpp              # Social Force Model
│   │   ├── fuzzy.cpp            # Fuzzy Logic Controller (Sugeno)
│   │   ├── lanekeepingassist.cpp# Lane Keeping Assist dengan EMA + Deadzone
│   │   ├── lidar_scan_node.cpp  # Pemrosesan data LiDAR
│   │   ├── data_log.cpp         # Logger data pengujian (CSV)
│   │   ├── object_tracker.cpp   # Pelacak objek
│   │   ├── encoder_read.cpp     # Pembaca encoder roda
│   │   ├── dashboard.cpp        # Tampilan dashboard ROS
│   │   ├── keyboard_handler.cpp # Kendali manual via keyboard
│   │   └── ...
│   ├── scripts/                 # Python scripts (simulasi & visualisasi)
│   │   ├── new_reverse.py       # Script navigasi utama
│   │   ├── rplidar.py           # Interface RPLiDAR
│   │   ├── pid_tuner.py         # Tuner PID interaktif
│   │   ├── cop_lidar_anim.py    # Animasi visualisasi LiDAR
│   │   ├── lka_logger.py        # Logger Lane Keeping
│   │   └── ...
│   ├── launch/                  # File launch ROS
│   ├── coppelia/                # Simulasi CoppeliaSim
│   ├── CMakeLists.txt
│   └── package.xml
│
├── yolov8_seg_ros/              # Package deteksi objek YOLOv8
│   ├── src/                     # Node C++ untuk inference
│   ├── include/
│   └── launch/
│
├── time_sync_pkg/               # Package sinkronisasi waktu
│   ├── src/
│   ├── msg/                     # Custom ROS messages
│   └── launch/
│
├── camera_access/               # Package akses kamera
│   ├── src/
│   └── launch/
│
├── rplidar_ros/                 # Driver RPLiDAR (submodule)
│   ├── src/
│   ├── sdk/
│   └── launch/
│
├── Hasil Pengujian/             # Data CSV hasil pengujian (CSV only)
│
├── analyze_all.py               # Analisis komprehensif semua data
├── generate_plots.py            # Generator grafik hasil pengujian
├── compare_all_runs.py          # Perbandingan antar run pengujian
├── inspect_dz_05_errors.py      # Inspeksi error deadzone
├── check_all_rho.py             # Validasi parameter rho SFM
└── lka_visualization.ipynb      # Notebook visualisasi LKA
```

---

## ⚙️ Persyaratan Sistem

### Hardware
- **Komputer Onboard**: Raspberry Pi 4 / Jetson Nano (atau setara)
- **LiDAR**: RPLiDAR A1/A2
- **Kamera**: USB Webcam / CSI Camera (resolusi min. 640×480)
- **Platform**: Kendaraan Ackerman dengan motor DC + driver PWM
- **Encoder**: Encoder roda untuk odometri

### Software
| Komponen | Versi |
|---|---|
| Ubuntu | 20.04 LTS |
| ROS | Noetic |
| Python | 3.8+ |
| OpenCV | 4.x |
| TensorRT | 8.x (untuk YOLOv8) |
| NumPy / Matplotlib | Latest |

---

## 🚀 Instalasi & Build

### 1. Clone Repository
```bash
cd ~/catkin_ws/src
git clone https://github.com/MrFuther/AckermanSFM.git
```

### 2. Install Dependensi ROS
```bash
sudo apt-get install ros-noetic-cv-bridge \
                     ros-noetic-image-transport \
                     ros-noetic-sensor-msgs \
                     ros-noetic-geometry-msgs
```

### 3. Build Workspace
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

---

## ▶️ Menjalankan Sistem

### Mode Penuh (semua modul aktif)
```bash
# Terminal 1: ROS Master
roscore

# Terminal 2: LiDAR
roslaunch rplidar_ros rplidar.launch

# Terminal 3: Kamera + YOLOv8
roslaunch yolov8_seg_ros yolov8_seg.launch

# Terminal 4: Package utama (SFM + LKA + Fuzzy + Drive)
roslaunch ackerman ackerman.launch
```

### Mode Pengujian LKA Saja
```bash
roslaunch ackerman lka_test.launch
```

### Visualisasi Data LiDAR (Python)
```bash
python3 ackerman/scripts/cop_lidar_anim.py
```

---

## 🧠 Algoritma Utama

### Social Force Model (SFM)
Model fisika berbasis gaya sosial untuk mengatur jarak aman antar kendaraan. Menggunakan parameter **proxemik dinamis** yang dihitung secara otomatis berdasarkan kecepatan relatif dan jarak obstacle.

```
F_total = F_drive + F_repulsive + F_attractive
```

### Fuzzy Logic Controller (Sugeno)
Kontroler fuzzy dengan **2 input** dan **1 output**:
- **Input 1**: Jarak ke obstacle (`distance`) — 4 himpunan: *Intimate, Personal, Social, Public*
- **Input 2**: Perubahan jarak (`Δdistance`) — 3 himpunan: *Mendekat, Stabil, Menjauh*
- **Output**: Gain akselerasi [0..1] — 12 aturan inferensi

### Lane Keeping Assist (LKA)
Deteksi marka jalan dengan pipeline:
1. **ROI Masking** → 2. **Filter HLS** (warna putih) → 3. **Canny Edge** → 4. **Hough Transform**
5. **Adaptive Lane Width Memory** → 6. **EMA Filter** (α=0.2) → 7. **Deadzone** (±30px)

---

## 📊 Hasil Pengujian

### Parameter Optimal LKA

| Parameter | Nilai Optimal |
|---|---|
| EMA α | 0.4 |
| Deadzone | 40 px |
| Lookahead (target_y) | 300 px |

### Performa SFM

| Skenario | Jarak Pengereman | RMSE |
|---|---|---|
| Objek Diam (Person) | 0.8 m | — |
| Objek Bergerak (Bus) | 1.2 m | — |
| Truck (Kmult 1.5) | 1.0 m | — |

> Data lengkap ada di folder `Hasil Pengujian/` (file CSV)  
> Grafik ada di root folder (file `.png` — dikecualikan dari repo, generate ulang dengan `generate_plots.py`)

---

## 📈 Visualisasi & Analisis

```bash
# Generate semua grafik hasil pengujian
python3 generate_plots.py

# Analisis komprehensif
python3 analyze_all.py

# Bandingkan semua run
python3 compare_all_runs.py

# Buka notebook LKA
jupyter notebook lka_visualization.ipynb
```

---

## 🗺️ ROS Topics

| Topic | Tipe | Deskripsi |
|---|---|---|
| `/camera` | `sensor_msgs/Image` | Frame kamera raw |
| `/lane_error` | `std_msgs/Float32` | Error posisi lateral (px) |
| `/near_distance` | `std_msgs/Float32MultiArray` | [distance, delta_distance] |
| `/fuzzy` | `std_msgs/Float32` | Output gain fuzzy [0..1] |
| `/vehicle_speed` | `std_msgs/Float64` | Kecepatan kendaraan (m/s) |
| `/lka/output_image` | `sensor_msgs/Image` | Visualisasi LKA |
| `/lka/mask_image` | `sensor_msgs/Image` | Mask marka jalan |

---

## 📄 Lisensi

Proyek ini dibuat untuk keperluan akademis. Lisensi akan ditentukan kemudian.

---

## 👤 Kontak

**Azra Pralingga**  
Sarjana Terapan Teknik Komputer PENS 
📧 [GitHub: MrFuther](https://github.com/MrFuther)
