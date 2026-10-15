# Walkthrough: Pengujian Performa LKA & Data Logger FPS

Kami telah membuat alat pengukur performa (data logger) dan mengintegrasikannya ke dalam peluncuran sistem. Ikuti langkah-langkah di bawah ini untuk memverifikasi apakah penurunan performa LKA memang disebabkan oleh drop FPS dari `/camera_sync`.

---

## 1. Perubahan yang Dilakukan

1. **Membuat Data Logger**: [lka_logger.py](file:///a:/ACKERMAN/ackerman/scripts/lka_logger.py)
   *   Berlangganan ke topik `/lane_error` (yang terbit setiap kali LKA memproses satu frame citra).
   *   Menghitung **FPS Instan** dan **FPS Rata-rata** menggunakan sliding window 10 sampel terakhir.
   *   Menyimpan riwayat data ke file CSV berformat `lka_performance_log_HH_MM_SS_DD-MM-YYYY.csv` langsung di folder root workspace (`a:/ACKERMAN`).

2. **Integrasi Launch File**: [integration.launch](file:///a:/ACKERMAN/ackerman/launch/integration.launch#L43-L45)
   *   Menambahkan node `lka_fps_logger` secara otomatis saat menjalankan sistem utama.

---

## 2. Cara Menguji Kinerja (Panduan Langkah Demi Langkah)

### Langkah A: Pastikan Skrip Logger Memiliki Izin Eksekusi
Buka terminal pada host Linux (Jetson Nano) Anda di folder `ackerman/scripts`, dan jalankan:
```bash
chmod +x lka_logger.py
```

---

### Skenario Uji 1: LKA Menggunakan `/camera_sync` (Kondisi Saat Ini)
1. **Jalankan Sistem Utama**:
   ```bash
   roslaunch ackerman integration.launch
   ```
2. **Amati Logger di Terminal**:
   Skrip `lka_fps_logger` akan mencetak rata-rata FPS setiap 1 detik ke terminal:
   ```text
   [INFO] [LKA Log] Error:  23.45 | FPS: 11.20 Hz
   ```
3. **Analisis File CSV**:
   Setelah Anda menghentikan launch dengan `Ctrl + C`, periksa file CSV baru di folder root `a:/ACKERMAN/lka_performance_log_*.csv`. Anda akan melihat kolom FPS rata-rata tertahan di kisaran **~11 FPS** (membuktikan pembatasan sinkronisasi).

---

### Skenario Uji 2: Meningkatkan LKA ke `/camera` (30 FPS Tanpa Hambatan)
Untuk melepas limit FPS dan mengalirkan data kamera langsung ke LKA:

1. **Ubah Sumber Topik di LKA**:
   Buka file [lanekeepingassist.cpp](file:///a:/ACKERMAN/ackerman/src/lanekeepingassist.cpp#L34). 
   Ubah baris:
   ```cpp
   image_sub = nh.subscribe("/camera_sync", 1, &LaneKeepingNode::imageCallback, this);
   ```
   Menjadi:
   ```cpp
   image_sub = nh.subscribe("/camera", 1, &LaneKeepingNode::imageCallback, this);
   ```

2. **Kompilasi Ulang Workspace**:
   Masuk ke root direktori catkin workspace Anda di terminal linux, lalu kompilasi ulang node C++:
   ```bash
   catkin_make
   ```

3. **Jalankan Kembali & Bandingkan**:
   Jalankan kembali launch file:
   ```bash
   roslaunch ackerman integration.launch
   ```
   Amati output terminal dari logger. Rata-rata FPS seharusnya melonjak menjadi **~30 FPS** (atau batas kemampuan maksimum hardware CPU/OpenCV dalam memproses citra). Lintasan robot seharusnya menjadi jauh lebih mulus karena respons kemudi diperbarui 3 kali lebih cepat!
