# Rencana Implementasi Proxemics Robot Dinamis & Optimasi LKA

Dokumen ini menjelaskan rencana perubahan kode untuk mengimplementasikan *Smooth Deadzone* pada LKA dan penyesuaian parameter *Proxemics* (elips keamanan) robot Ackerman secara dinamis berdasarkan kecepatan target ($v_0$) antara 0.3 m/s hingga 1.0 m/s.

## User Review Required

> [!IMPORTANT]
> **Skema Interpolasi Proxemics Robot**:
> Kami mengusulkan interpolasi linier untuk ukuran elips keamanan robot (*semi-axes* $a$ dan $b$) berdasarkan kecepatan target $v_0$:
> * Pada $v_0 \le 0.3$ m/s (Lambat): $a = 0.5$ m, $b = 0.3$ m (Elips lebih kecil agar tidak berhenti terlalu cepat).
> * Pada $v_0 \ge 1.0$ m/s (Cepat): $a = 1.0$ m, $b = 0.5$ m (Elips standar untuk keamanan kecepatan tinggi).
> * Rentang $0.3 < v_0 < 1.0$ m/s: Interpolasi linier kontinu antara kedua batas tersebut.

## Proposed Changes

### 1. Sensor Fusion & SFM

#### [MODIFY] [sfmazra.cpp](file:///a:/ACKERMAN/ackerman/src/sfmazra.cpp)
* **Import Header Baru**: Menambahkan `#include "std_msgs/Float64.h"` untuk menangani topik `/v0_target`.
* **Subscriber Baru**: Menambahkan subscriber ke topik `/v0_target` untuk memantau perubahan kecepatan target secara real-time.
* **Kalkulasi Proxemics Dinamis**: Di dalam fungsi `sfm()`, kita menghitung ukuran elips keamanan robot secara dinamis berdasarkan nilai $v_0$ sebelum menghitung `r_robot`.

```diff
+#include "std_msgs/Float64.h"
...
+float current_v0 = 1.0f;
+void v0Callback(const std_msgs::Float64::ConstPtr& msg) {
+    current_v0 = msg->data;
+}
...
 void sfm() {
     float nearest_distance = 10.0;
     float nearest_angle = 180.0;
     bool isOverlap = false;
     ObjectParameters params = getActiveParameters();
 
-    ...
-    float r_robot = calculateCoordinates(nearest_angle, params.semi_axis_a, params.semi_axis_b);          
+    // Kalkulasi Proxemics Robot Dinamis berdasarkan v0
+    float v0_clamped = std::max(0.3f, std::min(1.0f, current_v0));
+    float robot_a = 0.5f + (v0_clamped - 0.3f) * (1.0f - 0.5f) / (1.0f - 0.3f);
+    float robot_b = 0.3f + (v0_clamped - 0.3f) * (0.5f - 0.3f) / (1.0f - 0.3f);
+
+    float r_robot = calculateCoordinates(nearest_angle, robot_a, robot_b);          
     float r_obstacle = calculateCoordinates(nearest_angle + 180, params.semi_axis_a, params.semi_axis_b); 
```

---

### 2. Visualisasi GUI

#### [MODIFY] [animationnew.py](file:///a:/ACKERMAN/ackerman/scripts/animationnew.py)
* **Subscriber Baru**: Menambahkan subscriber ke `/v0_target` untuk memperbarui visualisasi elips keamanan robot.
* **Visualisasi Dinamis**: Mengubah *static ellipse* robot agar bentuknya berubah secara real-time di layar mencerminkan ukuran komputasi aktual di C++.

---

### 3. Lane Keeping Assist (LKA)

#### [MODIFY] [lanekeepingassist.cpp](file:///a:/ACKERMAN/ackerman/src/lanekeepingassist.cpp)
* **Peralihan ke /camera**: Mengubah pendaftaran subscriber kamera ke `/camera` (30 FPS).
* **Smooth Deadzone**: Menerapkan batas zona mati 30 piksel untuk memfilter getaran setir.

*(Poin LKA ini telah disetujui sebelumnya dan telah diimplementasikan).*

## Verification Plan

### Automated Tests
* Kompilasi ulang workspace menggunakan:
  ```bash
  catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3 -march=native -mfpu=neon"
  ```

### Manual Verification
1. Jalankan `roslaunch ackerman integration.launch`.
2. Ubah kecepatan target $v_0$ (misalnya melalui keyboard / controller) ke **0.3 m/s**:
   * Amati di layar visualizer bahwa elips garis putus-putus merah (Robot Proxemics) mengecil menjadi $a=0.5, b=0.3$.
   * Dekatkan objek ke robot dan pastikan robot baru mengerem/berhenti saat jarak objek sudah sangat dekat (tidak berhenti terlalu awal).
3. Ubah $v_0$ ke **1.0 m/s**:
   * Amati elips merah membesar kembali ke ukuran penuh ($a=1.0, b=0.5$) untuk memberikan ruang rem yang lebih aman.
