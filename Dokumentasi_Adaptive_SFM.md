Maaf atas kendala tersebut. Terkadang fitur unduhan file bisa mengalami gangguan.  
Sebagai solusi termudah, Anda bisa langsung menyalin (*copy*) seluruh teks dokumentasi di bawah ini. Anda bisa menempelkannya (*paste*) ke dalam Notepad, VS Code, atau langsung menggunakannya sebagai pengantar saat membuka sesi percakapan baru dengan saya.  
Berikut adalah isi lengkap dari **Dokumentasi\_Adaptive\_SFM.md**:

Markdown  
\# Dokumentasi Sistem Adaptive Social Force Model (SFM) untuk Navigasi Robot Otonom

Dokumentasi ini merangkum seluruh spesifikasi arsitektur, formulasi matematika, analisis parameter, serta panduan penyetelan (*\*tuning\**) untuk sistem **\*\*Adaptive Social Force Model (SFM)\*\*** yang telah dibahas dan dikembangkan.

\---

\#\# 1\. Arsitektur & Komponen Sistem

Sistem ini mengubah pendekatan navigasi robot konvensional (yang hanya menganggap rintangan sebagai titik geometris) menjadi sistem yang **\*\*sadar konteks (*\*context-aware\**)\*\***. Robot dapat mengidentifikasi jenis objek di sekitarnya dan mengubah perilaku tolak-menolak secara dinamis berdasarkan karakteristik fisik dan sosial objek tersebut.

\#\#\# A. Perubahan pada \`mainnew.cpp\` (Object Detection & Tracking)  
\* **\*\*Fitur Utama:\*\*** Mendeteksi objek terdekat menggunakan model visi (YOLOv8-Seg) dalam rentang sudut pengamatan depan $\[-150^\\circ, 150^\\circ\]$.  
\* **\*\*Publisher ROS:\*\*** \`/active\_object\_type\` (\`std\_msgs::Int32\`) — mengirimkan ID objek terdekat yang aktif. Jika tidak ada objek dalam jangkauan, dikirimkan nilai \`-1\`.  
\* **\*\*Pemetaan ID Objek:\*\***  
  \* \`0\`: Bus  
  \* \`1\`: Car (Mobil)  
  \* \`2\`: Motorcycle (Sepeda Motor)  
  \* \`3\`: Person (Pejalan Kaki)  
  \* \`4\`: Truck

\#\#\# B. Perubahan pada \`sfm.cpp\` (Inti Komputasi)  
\* **\*\*Fitur Utama:\*\*** Membaca ID dari topik \`/active\_object\_type\` dan secara dinamis mengubah parameter elips keamanan serta penguatan gaya (*\*gain multiplier\**).  
\* **\*\*Struktur Data Parameter:\*\***  
  \`\`\`cpp  
  struct ObjectParameters {  
      float semi\_axis\_a;    // Ukuran zona horizontal elips  
      float semi\_axis\_b;    // Ukuran zona vertikal elips  
      float psi;            // Decay rate (tingkat peluruhan) gaya sosial  
      float k\_multiplier;   // Pengganda gain kekuatan respons  
      std::string name;  
  };

### **C. Perubahan pada animation.py (Umpan Balik Visual)**

* **Fitur Visual:** Menggambar elips adaptif di sekitar robot secara real-time yang ukuran dan warnanya berubah sesuai dengan objek aktif yang sedang direspons.  
* **Kode Warna Visualisasi:**  
  * 🔴 **Bus** \- Merah  
  * 🟠 **Truck** \- Merah Tua (*Dark Red*)  
  * 🟡 **Car** \- Oranye  
  * 🟢 **Motorcycle** \- Kuning  
  * 🟢 **Person** \- Hijau  
* **Display Info:** Menampilkan teks nama objek aktif dan ukuran elips saat ini $(a, b)$ pada layar operator.

## **2\. Formulasi Matematika SFM Adaptif**

Modifikasi dilakukan pada komponen gaya sosial dan gaya fisik tradisional dengan memasukkan variabel dinamis hasil ekstraksi dari basis data parameter objek aktif.

### **A. Gaya Sosial Adaptif ($f\_{social}$)**

Gaya sosial merepresentasikan "rasa takut" atau kecenderungan robot untuk menjauh sebelum terjadi kontak fisik.

$$f\_{social} \= k\_{effective} \\cdot \\exp\\left(\\frac{r\_{value} \- \\text{distance}}{\\psi}\\right)$$  
Dimana nilai penguatan efektif ($k\_{effective}$) dihitung dari:

$$k\_{effective} \= k\_{fuzzy} \\cdot k\_{multiplier}$$

* **$k\_{fuzzy}$**: Nilai dasar penguatan yang dihasilkan oleh logika kendali fuzzy robot.  
* **$k\_{multiplier}$**: Nilai pengali spesifik objek (menguatkan respons untuk kendaraan besar, melemahkan untuk pejalan kaki).  
* **$\\psi$ (Psi)**: *Decay rate* yang mengatur jangkauan pengaruh gaya.

### **B. Gaya Fisik Adaptif ($f\_{physical}$)**

Gaya fisik bekerja ketika zona aman elips robot telah terobos oleh objek untuk menghindari benturan keras.

$$f\_{physical} \= k\_{effective} \\cdot (r\_{value} \- \\text{distance})$$

### **C. Perhitungan Geometri Elips Adaptif**

Jari-jari referensi elips aman ($r\_{value}$ atau $r\_{robot}$) dihitung secara dinamis berdasarkan sudut datang objek terhadap robot:

$$\\text{float } r\_{robot} \= \\text{calculateCoordinates}(\\text{angle}, a\_{adaptif}, b\_{adaptif})$$

## **3\. Analisis & Koreksi Parameter**

### **Evaluasi Struktur Awal**

Pada rancangan awal, seluruh objek memiliki nilai semi\_axis\_a \= 1.0f. Ini diidentifikasi sebagai **masalah keselamatan kritis (*safety issue*)**. Bus atau truk memiliki dimensi fisik yang jauh lebih besar daripada pejalan kaki. Jika batas geometri horizontal ($a$) disamakan, robot akan berada terlalu dekat dengan badan kendaraan besar, meningkatkan risiko tabrakan linear maupun gesekan lambung.

### **Masalah *Jumping Force* (Sentakan Gerakan)**

Jika klasifikasi objek dari node visi mengalami fluktuasi cepat antar-frame (misal: mendeteksi *Car* lalu berubah menjadi *Truck* secara instan), nilai $k\_{effective}$ akan melompat dari 1.2f ke 1.4f. Hal ini menyebabkan perubahan gaya yang drastis secara tiba-tiba (*jerk*), membuat pergerakan kemudi atau pengereman robot menjadi kasar.

* **Solusi Korektif:** Disarankan mengimplementasikan *low-pass filter* atau *exponential smoothing* sederhana pada transisi parameter antar frame di dalam sfm.cpp.

## **4\. Panduan Penyetelan Parameter (*Tuning Guide*)**

### **A. Cara Mencari Nilai Decay Rate ($\\psi$)**

$\\psi$ mengontrol gradien penurunan gaya sosial terhadap jarak. Secara matematis, gaya akan meluruh menjadi sekitar $36.8\\%$ dari kekuatan maksimumnya saat $(\\text{distance} \- r\_{value}) \= \\psi$.

* **Metode Tuning:** Dekatkan robot ke objek statis. Perhatikan titik jarak di mana kecepatan robot mulai berkurang secara halus.  
* **Gejala $\\psi$ Terlalu Besar:** Robot bersikap terlalu paranoid, memperlambat laju dari jarak yang sangat jauh padahal jalur masih aman (tidak efisien).  
* **Gejala $\\psi$ Terlalu Kecil:** Robot bersikap terlalu agresif (*reckless*), melaju kencang mendekati objek lalu mengerem mendadak di detik terakhir.

### **B. Cara Mencari Nilai $k\_{multiplier}$**

$k\_{multiplier}$ mengontrol magnitudo atau amplitudo mutlak dari gaya tolak saat objek memasuki perimeter aman.

* **Metode Tuning:** Jalankan robot dalam skenario lintasan lurus menuju objek.  
* **Gejala $k$ Terlalu Besar:** Robot mengalami osilasi (bergetar hebat) atau berhenti mendadak secara kasar (*jerking motion*) begitu mendeteksi objek.  
* **Gejala $k$ Terlalu Kecil:** Robot mengabaikan batas elips keamanan dan menerobos zona aman hingga terjadi tabrakan ringan (*soft collision*).

## **5\. Rekomendasi Basis Data Parameter (Revisi Akhir)**

Tabel berikut merupakan hasil optimalisasi yang menyeimbangkan antara ukuran dimensi fisik kendaraan nyata dan karakteristik psikologis sosial navigasi:

| ID | Jenis Objek | a (Horizontal) | b (Vertikal) | ψ (Decay) | kmultiplier​ | Karakteristik Perilaku |
| :---- | :---- | :---- | :---- | :---- | :---- | :---- |
| **0** | **Bus** | 2.5 m | 1.2 m | 0.40 | 1.8x | Dimensi fisik masif, gaya tolak terkuat, pengeremen paling awal. |
| **1** | **Car** | 1.8 m | 0.9 m | 0.30 | 1.3x | Standar ukuran mobil menengah, respons pengereman moderat. |
| **2** | **Motorcycle** | 1.2 m | 0.6 m | 0.25 | 1.0x | Respons gesit, jarak toleransi lebih rapat dibandingkan mobil. |
| **3** | **Person** | 0.6 m | 0.4 m | 0.15 | 0.6x | Paling kompak, robot diizinkan mendekat tanpa pengereman berlebih. |
| **4** | **Truck** | 2.5 m | 1.3 m | 0.38 | 1.7x | Mirip seperti bus, zona elips vertikal disesuaikan dengan lebar muatan. |
| **\-** | **Default** | 1.0 m | 0.5 m | 0.20 | 1.0x | Digunakan ketika tidak ada objek spesifik terdeteksi di depan robot. |

### **Implementasi Kode C++ (Revisi Terbuka):**

C++  
// Database parameter hasil optimalisasi keselamatan dan kelancaran gerak  
std::vector\<ObjectParameters\> object\_params \= {  
    {2.5f, 1.2f, 0.40f, 1.8f, "Bus"},        // Index 0: Bus  
    {1.8f, 0.9f, 0.30f, 1.3f, "Car"},        // Index 1: Car  
    {1.2f, 0.6f, 0.25f, 1.0f, "Motorcycle"}, // Index 2: Motorcycle  
    {0.6f, 0.4f, 0.15f, 0.6f, "Person"},     // Index 3: Person  
    {2.5f, 1.3f, 0.38f, 1.7f, "Truck"}       // Index 4: Truck  
};

// Default parameters saat topik bernilai \-1  
ObjectParameters default\_params \= {1.0f, 0.5f, 0.20f, 1.0f, "Default"};

## **6\. Diagram Alur Kerja Sistem (Data Flow)**

Plaintext  
   \[ Object Detection Node \]  
              │  
              ▼ Deteksi: Bus berada di jarak 3.5m, sudut \-15°  
   \[ Publish: /active\_object\_type \= 0 \]  
              │  
              ▼ Diterima oleh SFM Node  
   \[ SFM Node: Memuat Parameter Index 0 \]  
              │ (a=2.5, b=1.2, psi=0.40, k\_mult=1.8)  
              ▼  
   \[ Hitung f\_social & f\_physical adaptif \]  
              │  
              ▼ Gaya tolak membesar secara eksponensial dari jarak jauh  
   \[ Robot Melakukan Pengereman/Pelepasan Gas Lebih Awal \]  
              │  
              ▼ Sinkronisasi visualisasi  
   \[ Animation Node: Merender Elips Merah Besar (2.5m x 1.2m) \]

## **7\. Instruksi Eksekusi Sistem (Deployment)**

Pastikan seluruh node berada di dalam workspace ROS Anda (misal: \~/ACKERMAN). Eksekusi perintah-perintah berikut pada terminal terpisah:

1. **Kompilasi Workspace:**  
   Bash  
   cd \~/ACKERMAN  
   catkin\_make  
   source devel/setup.bash

2. **Terminal 1 \- Node Deteksi Objek & Segmentasi:**  
   Bash  
   rosrun yolov8\_seg\_ros mainnew

3. **Terminal 2 \- Node Inti Komputasi SFM:**  
   Bash  
   rosrun your\_package sfm\_node

4. **Terminal 3 \- Node Visualisasi Animasi:**  
   Bash  
   rosrun your\_package animation.py

5. **Terminal 4 \- Monitoring Topik Aktif (Debugging):**  
   Bash  
   rostopic echo /active\_object\_type  
