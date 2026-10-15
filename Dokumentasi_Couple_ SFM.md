

Python  
md\_content \= """\# Dokumentasi Pengembangan Sistem Robotika Otonom (ADAS)  
\*\*Pengembang\*\*: Azra Pralingga Sakti  
\*\*Fokus Utama\*\*: Integrasi \*Lane Keeping Assist\* (LKA) dan \*Adaptive Cruise Control\* (ACC) berbasis \*Coupled Social Force Model\* (SFM) di dalam ekosistem ROS.

\---

\#\# 1\. Arsitektur Visi Komputer & Lane Keeping Assist (\`lanekeepingassist.cpp\`)  
Pengembangan \*node\* visi difokuskan pada kestabilan pembacaan garis marka jalan dan kehalusan respons titik target (hijau) agar sistem kemudi mekanis tidak cepat aus.

\#\#\# A. \*Lookahead Distance\*  
Mengubah titik kalkulasi garis tengah (\*center point\*) dari dasar layar (tepat di bawah bumper) menjadi lebih tinggi / jauh ke depan (\`Y\_target \= 300\`). Hal ini memungkinkan robot mengantisipasi belokan lebih awal dan mengurangi \*understeer\*.

\#\#\# B. \*Dynamic Lane Width Fallback\*  
Memperbaiki logika tebakan garis saat robot berada di tikungan dan salah satu marka hilang. Menggunakan perhitungan \*\*Lebar Lajur Dinamis\*\* yang selalu diperbarui via \*Exponential Moving Average\* (EMA) untuk mencegah robot membanting setir terlalu jauh akibat \*Perspective Distortion\*.

Output kode

File generated successfully: Dokumentasi\_Sistem\_Otonom\_Azra.md

\`\`\`cpp  
// Update memori lebar jalan secara dinamis (EMA 10% data baru, 90% data lama)  
last\_known\_width \= (0.1 \* current\_width) \+ (0.9 \* last\_known\_width);

### **C. *Low-Pass Filter* (EMA) pada Titik Target**

Meredam *jitter* (getaran frekuensi tinggi) dari hasil deteksi piksel *OpenCV* (cv::HoughLinesP).

C++  
float smoothed\_lane\_center \= (alpha\_filter \* raw\_lane\_center) \+ ((1.0 \- alpha\_filter) \* prev\_center\_x);

## **2\. Sistem Kendali Motor & Servo (driveazra.cpp)**

*Node* ini bertanggung jawab untuk menerjemahkan nilai *error* navigasi ke dalam sinyal PWM mekanis secara halus.

### **A. Koreksi Arah Putaran Servo (*Positive Feedback Loop*)**

Mengatasi fenomena kemudi terbalik dengan melakukan inversi nilai *error* secara matematis (error \= \-error;). Ini mencegah robot merespons deviasi lajur dengan banting setir ke arah yang salah yang memicu *ping-pong effect*.

### **B. *Slew Rate Limiter* (Pembatas Langkah)**

Mencegah servo menerima perintah perubahan PWM yang ekstrem dalam satu waktu. Menggunakan step\_size \= 15.0 (pada \~30 FPS) untuk memaksa servo bergerak mulus dan bertahap menuju target PWM (*damping mechanism*).

## **3\. Integrasi Navigasi: *Coupled Social Force Model* (sfmazra.cpp)**

Memodifikasi algoritma SFM klasik menjadi sistem penggerak otonom terpadu (*Coupled*) yang mengawinkan sumbu akselerasi (X) dan sumbu kemudi (Y) menggunakan prinsip **Superposisi Vektor 2D**.

### **A. Komponen Gaya**

* **$F\_{goal}$ (Gaya Maju):** Dorongan konstan ke arah Sumbu X ($50.0$) untuk melaju.  
* **$F\_{lane}$ (Gaya Marka):** Tarikan lateral (Kiri/Kanan Sumbu Y) hasil kalkulasi *Fuzzy Logic* LKA.  
* **$F\_{obs}$ (Gaya Tolak Objek):** Resistensi psikologis dan fisik berdasarkan elips proksemik objek (manusia, mobil, bus, dll).

### **B. Dekomposisi & Vektor Resultan**

Gaya dari rintangan dipecah menjadi komponen X dan Y menggunakan perhitungan trigonometri murni (Sinus dan Cosinus).

C++  
// Dekomposisi gaya tolak berlawanan arah dari rintangan  
float F\_obs\_x \= \-f\_force\_static \* cos(obs\_theta\_rad);   
float F\_obs\_y \= \-f\_force\_static \* sin(obs\_theta\_rad); 

// Superposisi Vektor Total  
float F\_total\_x \= F\_GOAL\_X \+ F\_obs\_x;  
float F\_total\_y \= F\_lane\_y \+ F\_obs\_y;

### **C. Transformasi Output**

* **Akselerasi/Rem ($F\_{nav}$):** Kalkulasi selisih $F\_{goal\\\_x} \- F\_{total\\\_x}$.  
* **Sudut Kemudi ($\\rho$):** Kalkulasi rasio vektor lateral terhadap longitudinal menggunakan tangen dua kuadran atan2(Y, X). Memungkinkan robot melakukan manuver *dodging* (menghindar) saat menemui halangan.

## **4\. Optimasi Komputasi & Visualisasi UI**

Melakukan restrukturisasi antarmuka pengguna untuk memangkas *overhead* komputasi (CPU/GPU) pada *Single Board Computer* (Jetson Nano).

### **A. *Dashboard Node* (dashboard\_node.cpp)**

Menghentikan fungsi cv::imshow() yang dieksekusi secara sporadis di tiap *node* (LKA, YOLO, Masking). Menggunakan skema *Publisher-Subscriber* matriks gambar untuk menyatukan semua tampilan ke dalam **1 jendela grid gabungan** via cv::hconcat dan cv::vconcat.

### **B. Visualizer SFM Adaptif (animationnew.py)**

Meningkatkan kapabilitas skrip Python *Matplotlib* untuk menggambar elips proksemik adaptif (sesuai kelas YOLO) dan memvisualisasikan 4 vektor gaya secara *real-time* untuk kebutuhan diagnostik dan *debugging* parameter SFM.

## **5\. Antarmuka Kendali Manual/Otonom (keyboard\_handler.cpp)**

Memodifikasi antarmuka *teleoperation* berbasis SDL2 untuk menyertakan mode diagnostik modular.

* **Tombol T:** *Toggle* untuk mengaktifkan/mematikan fitur akselerasi otonom (Adaptive Cruise Control).  
* **Tombol Y:** *Toggle* independen untuk mengaktifkan/mematikan fitur kemudi otonom (Lane Keeping Assist).  
  Memungkinkan operator melakukan *test-drive* mekanis pada salah satu sumbu gerak tanpa intervensi sistem otonom di sumbu lainnya.  
  """

with open("Dokumentasi\_Sistem\_Otonom\_Azra.md", "w") as f:  
f.write(md\_content)  
print("File generated successfully: Dokumentasi\_Sistem\_Otonom\_Azra.md")

Pekerjaan yang sangat hebat hari ini, Mas Azra\! Saya telah merangkum seluruh arsitektur, pemecahan masalah, dan logika matematis yang kita kerjakan pada percakapan ini ke dalam satu \*file\* Markdown (\`.md\`) yang rapi. 

Dokumen ini sangat cocok Anda gunakan sebagai bahan rujukan (\*draft\*) untuk menyusun Bab 3 (Metodologi) atau Bab 4 (Analisis Sistem) pada buku Proyek Akhir Anda nanti.

Your MD file is ready:  
\[file-tag: code-generated-file-0-1781106906494711710\]

Jika Anda membutuhkan penjabaran lebih dalam untuk bagian tertentu (misalnya algoritma \*YOLO Bounding Box\* yang kita proyeksikan ke jarak LiDAR), atau butuh diagram tambahan, jangan ragu untuk memberi tahu saya. Selamat melanjutkan pengujian di lapangan\! Saya \*standby\* menunggu hasil manuver robot Anda\! 🚀  
