# Panduan Pengujian & Eksperimen Sistem ADAS Ackerman
## Kerangka Acuan Penulisan BAB IV: Eksperimen dan Analisis
**Penyusun**: Azra Pralingga Sakti (3222600053)

---

## BAB IV: EKSPERIMEN DAN ANALISIS

Bab ini menyajikan parameter uji, klasifikasi variabel penelitian, metodologi eksperimen, hasil pengujian, serta analisis terhadap sifat dan karakteristik respon sistem keselamatan aktif (LKA dan Coupled SFM) pada prototipe robot Ackerman.

### 4.1 VARIABEL PENELITIAN
Untuk menjamin validitas ilmiah dalam eksperimen ini, variabel-variabel penelitian didefinisikan secara ketat sebagai berikut:

1. **Variabel Bebas (*Independent Variables*)**:
   Variabel yang sengaja diubah/dimanipulasi untuk melihat pengaruhnya terhadap sistem:
   * **`deadzone`** LKA (Nilai: $0$ piksel, $15$ piksel, $30$ piksel, dan $50$ piksel).
   * Kecepatan target awal **$v_0$** (Nilai: $0.3\text{ m/s}$, $0.5\text{ m/s}$, $0.7\text{ m/s}$, dan $1.0\text{ m/s}$).
   * Posisi lateral objek rintangan memotong lajur (Nilai offset: $10\text{ cm}$, $20\text{ cm}$, dan $30\text{ cm}$ dari tepi lajur kiri).
   * **Koefisien perataan `alpha_filter`** EMA (Nilai: $1.0$ [tanpa filter], $0.5$, $0.2$ [default], dan $0.05$).

2. **Variabel Terikat (*Dependent Variables*)**:
   Variabel respon yang diukur dan diamati akibat perubahan variabel bebas. Seluruh variabel respon digital direkam secara terpusat oleh [data_log_integrasi.cpp](file:///a:/ACKERMAN/ackerman/src/data_log_integrasi.cpp) ke dalam file `data_integrasi_*.csv` saat robot berjalan dalam mode otonom:

   * **Data Telemetri Eksperimen** (dicatat dalam file log `data_integrasi_*.csv`):
     * **Kecepatan Aktual Kendaraan** (`Real Velocity (m/s)` via topik `/vehicle_speed`).
     * **Gaya Pengereman Virtual** (`Fnav (Force)` via topik `/fnav` dalam Newton).
     * **Jarak terhadap Objek/Rintangan** (`Distance (m)` via topik `/near_distance` dalam meter).
     * **Perintah Kecepatan ADAS** (`Velocity Command (m/s)` via topik `/velopub` dalam m/s).
     * **Output Kecepatan ke Motor** (`Velocity Vt (PWM)` via topik `v1` sebagai nilai integer kontrol aktuator).
     * **ID Objek Aktif** (`Active Object ID` via topik `/active_object_type` untuk melacak status deteksi rintangan).
     * **Simpangan Kesalahan Lajur Nyata** (`Lane Error (px)` via topik `/lane_error` dalam piksel).
     * **Kinerja Pemrosesan Frame LKA** (`LKA Instant FPS` & `LKA Avg FPS` dalam Hz).

   * **Variabel Fisik & Geometris Tambahan** (diperoleh melalui kalkulasi atau observasi langsung):
     * **Osilasi Sudut Kemudi** (`/rho` dalam derajat, diukur dari sudut roda depan/servo kemudi).
     * **Jarak Akhir Berhenti Nyata** ($d_{\text{stop}}$ dalam meter, diukur secara fisik dari bumper depan robot ke objek rintangan setelah berhenti penuh).
     * **Waktu Tunda Respons Sudut Kemudi** (*Steering Response Delay* dalam milidetik/ms, dihitung dari selisih waktu terbitnya simpangan kesalahan lajur hingga respon putar servo kemudi).

3. **Variabel Kontrol (*Control Variables*)**:
   Variabel yang dijaga konstan selama pengujian agar tidak memengaruhi hasil:
   * Resolusi input kamera ($640 \times 480$ piksel).
   * Lebar lajur uji fisik ($60\text{ cm}$).
   * Intensitas cahaya ruangan ($\approx 150\text{ Lux}$ - dikontrol menggunakan pencahayaan konstan laboratorium sesuai batasan masalah).
   * Nilai penguatan PID Aktuator (PID Kecepatan dan Kemudi tetap konstan).
   * Dimensi fisik objek rintangan (tipe rintangan: pejalan kaki / *Person*).

---

### 4.2 PARAMETER STANDAR EKSPERIMEN
Parameter dasar yang dikonfigurasi pada sistem pengendali diatur sebagai berikut:
* **Lookahead Distance (`Y_target`)**: $300$ piksel dari atas layar.
* **Batas Kemudi Maksimum (PWM)**: Rentang $1850$ s.d. $2250$ (tengah/lurus pada $2050$).
* **Faktor Skala Gaya SFM**: $F_{\text{goal\_x}} = 50.0\text{ N}$ dan $MAX\_F\_LANE = 40.0\text{ N}$.

---

### 4.3 TEMPAT UJI COBA
* **Area Uji**: Lorong lantai gedung Laboratorium dengan permukaan datar berbahan granit gelap untuk kontras maksimum terhadap marka.
* **Konfigurasi Lajur**: Jalur lurus sepanjang $10\text{ meter}$ dan tikungan dengan radius belok $\approx 1.5\text{ meter}$.
* **Marka Jalan**: Lakban putih solid berukuran lebar $5\text{ cm}$ yang ditarik sejajar sebagai garis pembatas lajur kiri dan kanan dengan lebar bersih $\approx 60\text{ cm}$.

---

### 4.4 WAKTU UJI COBA
Eksperimen dilakukan pada jam operasional laboratorium (pukul 10.00 hingga 17.00 WIB) dengan intensitas cahaya ruangan dikondisikan konstan pada tingkat $\approx 150\text{ Lux}$.

---

### 4.5 SPESIFIKASI PERALATAN UJI COBA
* **Komputer Utama**: Nvidia Jetson Nano (4GB RAM, Quad-core ARM A57 CPU).
* **Sensor Kamera**: Logitech C270 HD Webcam.
* **Sensor Jarak**: RPLiDAR A1 M8.
* **Pengendali Aktuator**: OpenRB-150.
* **Catu Daya**: Baterai LiPo 3S 11.1V (motor) & Powerbank 5V/3A (Jetson Nano).

---

### 4.6 ALAT PEREKAM DATA (LOGGING TOOL)
Untuk merekam variabel terikat secara terpusat, sinkron, dan objektif, sistem dilengkapi dengan satu modul logger terintegrasi:

* **Node Logger Integrasi (`data_log_integrasi`)**:
  * **Source Code**: [data_log_integrasi.cpp](file:///a:/ACKERMAN/ackerman/src/data_log_integrasi.cpp)
  * **File Output**: `data_integrasi_%H_%d-%m-%Y.csv` (tersimpan langsung di root workspace/folder eksekusi).
  * **Kondisi Perekaman (Trigger Otonom)**: Logger ini secara pintar hanya akan menuliskan baris data ke file CSV ketika robot berjalan dalam mode otonom penuh. Mode otonom aktif saat tombol `T` (Speed/ACC Command Toggle) **dan** tombol `Y` (Steering/LKA Toggle) diaktifkan bersamaan pada [keyboard_handler.cpp](file:///a:/ACKERMAN/ackerman/src/keyboard_handler.cpp). Perekaman akan otomatis ditangguhkan (*suspended*) jika robot dikembalikan ke kendali manual demi menjaga keaslian data pengujian otonom.
  * **Daftar Kolom & Pemetaan Variabel**:
    * `Timestamp`: Waktu pencatatan dengan presisi milidetik (`HH:MM:SS.mmm`).
    * `Velocity Vt (PWM)`: Sinyal perintah kecepatan motor DC (PWM) dari topik `v1`.
    * `Fnav (Force)`: Besarnya gaya pengereman virtual dari model Social Force Model (SFM) dari topik `/fnav`.
    * `Distance (m)`: Jarak terdekat robot ke rintangan yang terdeteksi dari topik `/near_distance`.
    * `Velocity Command (m/s)`: Perintah kecepatan target yang dihasilkan pengendali dari topik `/velopub`.
    * `Real Velocity (m/s)`: Kecepatan roda aktual robot berdasarkan pembacaan encoder dari topik `/vehicle_speed`.
    * `Active Object ID`: ID klasifikasi objek rintangan yang terdeteksi dari topik `/active_object_type`.
    * `Lane Error (px)`: Simpangan kesalahan lajur nyata yang dipublikasikan oleh LKA dari topik `/lane_error`.
    * `LKA Instant FPS`: FPS instan pengolahan citra LKA.
    * `LKA Avg FPS`: Rata-rata FPS pengolahan citra LKA dari sliding window 10 frame terakhir.

---

### 4.7 HASIL EKSPERIMEN (Saran Tabel & Grafik)

#### Eksperimen 1: Pengaruh Lebar Deadzone terhadap Kestabilan Sudut Kemudi LKA
Pengujian dilakukan pada lintasan lurus sepanjang $5\text{ meter}$ dengan kecepatan konstan $v = 0.5\text{ m/s}$.

* **Tabel Hasil Eksperimen 1**:
  | Variabel Bebas: Deadzone (piksel) | Standar Deviasi Lane Error (piksel) | Amplitudo Osilasi Sudut Kemudi /rho (derajat)* | Frekuensi Koreksi Setir (siklus/menit)** | Sifat Respon Setir |
  | :---: | :---: | :---: | :---: | :--- |
  | $0$ (Tanpa Deadzone) | $122.94$ | $27.96$ | $70.7$ | Sangat osilatif/bergetar |
  | $15$ | $88.84$ | $23.89$ | $54.9$ | Osilasi berkurang, setir aktif |
  | $30$ | $121.41$ | $26.65$ | $61.6$ | Stabil/Tenang di tengah lajur |
  | $50$ | $70.71$ | $19.02$ | $18.9$ | Lamban merespon Tikungan |

  > [!NOTE]
  > \* Amplitudo osilasi dihitung menggunakan standar deviasi dari sudut kemudi $\rho$ (derajat). Batas mekanis maksimum kemudi tersaturasi pada $77.32^\circ$ (rentang $141.34^\circ$ s.d. $218.66^\circ$).
  > \*\* Frekuensi koreksi dihitung berdasarkan jumlah siklus perubahan arah kemudi (melintasi garis tengah $180^\circ$) per menit.

* **Grafik Hasil Eksperimen 1**:
  ![Karakteristik Respon Lane Error terhadap Berbagai Lebar Deadzone LKA (v0 = 0.5 m/s)](file:///a:/ACKERMAN/deadzone_comparison_05ms.png)

#### Eksperimen 2: Karakteristik Respon Pengereman ACC Berdasarkan Variasi Kecepatan Target ($v_0$) dan Dynamic Proxemics
Pengujian dilakukan dengan menghadapkan robot pada pejalan kaki (*Person*) statis di tengah lajur.

* **Tabel Hasil Eksperimen 2**:
  | Kecepatan Target $v_0$ (m/s) | Parameter Dinamis Robot: semi-axis $a$ (m) | Jarak Deteksi Awal Mulai Mengerem (m) | Jarak Akhir Berhenti Nyata $d_{\text{stop}}$ (m) | Status Keselamatan (Tabrakan/Aman) |
  | :---: | :---: | :---: | :---: | :---: |
  | $0.3$ | $0.50$ | ... | ... | Aman (Berhenti presisi) |
  | $0.5$ | $0.64$ | ... | ... | Aman |
  | $0.7$ | $0.78$ | ... | ... | Aman |
  | $1.0$ | $1.00$ | ... | ... | Aman (Pengereman dini) |

* **Grafik yang Disarankan**: Grafik hubungan antara Kecepatan Target ($v_0$) terhadap Jarak Berhenti Nyata ($d_{\text{stop}}$).

#### Eksperimen 2B: Sifat Pengereman ACC Terpadu (Coupled SFM) dan Pengaruh K_multiplier (Context Aware Dynamic Proxemic)
Pengujian dilakukan pada lintasan lurus sepanjang $5\text{ meter}$ dengan kecepatan masuk otonom maksimum target $v_0 = 1.0\text{ m/s}$ (kecepatan aktual awal $\approx 1.03 - 1.05\text{ m/s}$) terhadap berbagai kelas rintangan (pejalan kaki, bus, dan truk) statis di tengah lajur untuk melihat pengaruh ukuran elips proksemik dan parameter $K_{multiplier}$ terhadap karakteristik deselerasi.

* **Tabel Hasil Eksperimen 2B**:
  | Tipe Rintangan | Kelas YOLO | Parameter Dinamis: Elips $a \times b$ (m) | Konstanta $K_{multiplier}$ | Kecepatan Awal Nyata (m/s) | Jarak Mulai Mengerem $d_{\text{start}}$ (m)* | Jarak Akhir Berhenti Nyata $d_{\text{stop}}$ (m)** | Gaya Repulsif Maksimum $F_{rep, max}$ (N) | Status Keselamatan |
  | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
  | Pejalan Kaki (Person) | Person (3) | $0.90 \times 0.15$ | $1.0$ | $1.052$ | $1.748$ | $0.546$ | $138.93$ | Aman |
  | Bus | Bus (0) | $1.20 \times 1.00$ | $1.5$ | $1.012$ | $1.820$ | $0.636$ | $263.42$ | Aman |
  | Truk ($K_{multiplier} = 1.5$) | Truck (4) | $1.20 \times 1.00$ | $1.5$ | $1.032$ | $1.826$ | $0.520$ | $399.43$ | Aman |
  | Truk ($K_{multiplier} = 2.0$) | Truck (4) | $1.20 \times 1.00$ | $2.0$ | $1.032$ | $1.894$ | $0.548$ | $484.20$ | Aman |

  > [!NOTE]
  > \* Jarak mulai mengerem ($d_{\text{start}}$) diukur ketika gaya repulsif $F_{rep}$ dari model SFM mulai bernilai non-zero ($F_{rep} > 1.0\text{ N}$) dan memicu perintah deselerasi kecepatan.
  > \*\* Jarak akhir berhenti nyata ($d_{\text{stop}}$) diukur secara relatif dari pembacaan sensor LiDAR (jarak bumper depan robot ke rintangan) setelah kendaraan berhenti penuh secara stabil.

* **Grafik Hasil Eksperimen 2B**:
  Visualisasi grafik komparatif profil pengereman (kecepatan, jarak rintangan, dan gaya repulsif $F_{rep}$ terhadap waktu relatif):

  ![Grafik Komparasi Profil Pengereman ACC Coupled SFM](file:///a:/ACKERMAN/sfm_braking_comparison.png)

  Berikut adalah profil pengereman otonom individual untuk masing-masing kelas objek:

  ````carousel
  ![Profil Pengereman: Pejalan Kaki (Person)](file:///a:/ACKERMAN/sfm_braking_person.png)
  <!-- slide -->
  ![Profil Pengereman: Bus](file:///a:/ACKERMAN/sfm_braking_bus.png)
  <!-- slide -->
  ![Profil Pengereman: Truk (K_mult = 1.5)](file:///a:/ACKERMAN/sfm_braking_truck_k15.png)
  <!-- slide -->
  ![Profil Pengereman: Truk (K_mult = 2.0)](file:///a:/ACKERMAN/sfm_braking_truck_k20.png)
  ````

#### Eksperimen 3: Karakteristik Kemudi Terpadu (Coupled LKA-SFM) pada Skenario Penghindaran Rintangan Samping
Pengujian dilakukan dengan meletakkan objek rintangan di sisi kiri dalam lajur yang sedikit memotong jalur lintasan robot.

* **Tabel Hasil Eksperimen 3**:
  | Offset Rintangan Memotong Lajur (cm) | Gaya Tolak Maksimum Objek $F_{\text{obs},y}$ (N) | Gaya Koreksi Maksimum LKA $F_{\text{lane},y}$ (N) | Nilai Maksimum Deviasi Lajur Kanan (piksel) | Keberhasilan Penghindaran (Ya/Tidak) |
  | :---: | :---: | :---: | :---: | :---: |
  | $10\text{ cm}$ | ... | ... | ... | Ya (Tetap dalam lajur) |
  | $20\text{ cm}$ | ... | ... | ... | Ya (Sedikit melipir ke kanan) |
  | $30\text{ cm}$ | ... | ... | ... | Tidak (Keluar marka kanan) |

#### Eksperimen 4: Pengaruh Parameter Smoothing (`alpha_filter`) terhadap Delay Respon Sudut Kemudi
Pengujian dilakukan saat robot berjalan pada kecepatan $0.5\text{ m/s}$ memasuki tikungan tajam lajur jalan dengan Deadzone 30 piksel.

* **Tabel Hasil Eksperimen 4**:
  | Nilai `alpha_filter` EMA | Karakteristik Kehalusan Kemudi | Waktu Mulai Setir Berbelok (*Response Delay*) | Nilai Maksimum *Overshoot* Kemudi | Status Keluar Lajur (Ya/Tidak) |
  | :---: | :--- | :---: | :---: | :---: |
  | $1.0$ (Tanpa Filter) | Sangat Kasar / Bergetar (Jitter Tinggi) | $0\text{ ms}$ (Instan) | Tinggi (Mengayun / Osilasi) | Tidak |
  | $0.8$ | Kasar (Getaran Ringan) | $\approx 30\text{ ms}$ | Sedang-Tinggi | Tidak |
  | $0.6$ | Cukup Halus (Osilasi Minim) | $\approx 70\text{ ms}$ | Sedang (Stabil) | Tidak |
  | $0.4$ | Sangat Halus / Mulus | $\approx 100\text{ ms}$ | Rendah | Tidak |
  | $0.2$ (Default) | Sangat Halus / Mulus (Respon Lambat) | $\approx 180\text{ ms}$ | Sangat Rendah | Tidak |

* **Grafik Hasil Eksperimen 4**:
  Visualisasi respon Lane Error terhadap waktu pada berbagai variasi filter EMA (v0 = 0.5 m/s, Deadzone = 30 px):
  
  ````carousel
  ![EMA 1.0 (Tanpa Filter)](file:///a:/ACKERMAN/ema_10_comparison.png)
  <!-- slide -->
  ![EMA 0.8 (Filter Sangat Ringan - Fallback)](file:///a:/ACKERMAN/ema_08_comparison.png)
  <!-- slide -->
  ![EMA 0.6 (Filter Sedang)](file:///a:/ACKERMAN/ema_06_comparison.png)
  <!-- slide -->
  ![EMA 0.4 (Filter Kuat)](file:///a:/ACKERMAN/ema_04_comparison.png)
  <!-- slide -->
  ![EMA 0.2 (Filter Sangat Kuat)](file:///a:/ACKERMAN/ema_02_comparison.png)
  ````

---

### 4.8 ANALISIS HASIL EKSPERIMEN

1. **Analisis Eksperimen 1 (Sifat Deadzone LKA)**:
   * Menganalisis bagaimana parameter *deadzone* mengubah sifat kontrol kemudi. Tanpa deadzone, pembacaan koordinat tengah lajur yang memiliki noise mikro langsung memicu aksi kemudi PID, menyebabkan setir bergetar terus-menerus dengan frekuensi koreksi tertinggi sebesar **70.7 siklus/menit**.
   * Nilai deadzone $30$ piksel memberikan keseimbangan optimal. Ia memadamkan respon koreksi setir terhadap noise kecil (menyaring *error* kecil menjadi $0.0$), namun tetap cukup responsif untuk membelokkan roda saat simpangan lajur melebihi toleransi (misalnya saat memasuki tikungan), dengan frekuensi koreksi stabil sebesar **61.6 siklus/menit**.
   * Jika deadzone terlalu besar ($50$ piksel), sistem memiliki kestabilan lateral yang sangat tinggi pada lintasan lurus (standar deviasi error terendah **70.71 px**), namun sifat respon setir menjadi sangat lamban dalam mengantisipasi tikungan karena zona toleransi yang terlalu lebar (frekuensi koreksi turun drastis menjadi hanya **18.9 siklus/menit**). Hal ini meningkatkan risiko robot meluncur keluar jalur sebelum koreksi kemudi dimulai.

2. **Analisis Eksperimen 2 (Sifat Dynamic Robot Proxemics)**:
   * Menganalisis hubungan antara kecepatan masuk ($v_0$) dengan ukuran elips keamanan robot ($a$). 
   * Pada kecepatan $0.3\text{ m/s}$, elips keamanan mengecil ($a = 0.5\text{ m}$). Secara fisis, hal ini membatasi gaya tolak sosial agar tidak aktif terlalu cepat, sehingga memperbolehkan robot mendekati objek dan berhenti pada jarak yang rapat dan presisi tanpa memicu pengereman dini yang tidak perlu.
   * Pada kecepatan $1.0\text{ m/s}$, elips keamanan membesar menjadi $a = 1.0\text{ m}$. Ini secara otomatis mengaktifkan pengereman ACC pada jarak yang lebih jauh, memberikan ruang deselerasi yang aman untuk menetralisir energi kinetik robot akibat inersia kecepatan tinggi.

2B. **Analisis Eksperimen 2B (Sifat Pengereman Coupled SFM dan Pengaruh K_multiplier)**:
    * **Perbedaan Karakteristik SFM Lama vs Coupled SFM**: Pada model SFM lama sebelum dikopel (*uncoupled*), pengereman dan kemudi bekerja secara terpisah dan tidak sinkron, di mana gaya navigasi arah maju ($F_{nav}$) tidak terpengaruh oleh dekomposisi gaya lateral. Setelah dikopel (*coupled*), gaya deselerasi (pengereman) dialihkan pada superposisi gaya sumbu longitudinal (akselerasi X):
      $$F_{\text{total}, x} = F_{\text{goal}} + F_{\text{obs}, x}$$
      di mana $F_{\text{obs}, x} = -f_{\text{force\_static}} \cos(\theta_{\text{obs}})$ adalah proyeksi gaya tolak sosial-fisik dari rintangan pada sumbu maju-mundur. Sinyal deselerasi virtual yang dikirimkan ke aktuator pengereman dipublikasikan sebagai gaya repulsif $F_{rep} = F_{\text{goal}} - F_{\text{total}, x} = -F_{\text{obs}, x}$.
    * **Pengaruh Tipe Rintangan terhadap Jarak Deteksi & Gaya Repulsif**: 
      * Untuk rintangan berisiko rendah seperti **Pejalan Kaki (Person)** ($K_{multiplier} = 1.0$), robot dapat mendekati objek lebih dekat sebelum mengerem, dengan $d_{\text{start}} = 1.748\text{ m}$, deselerasi yang dihasilkan halus dengan gaya maksimum puncak $F_{rep, max} = 138.93\text{ N}$, dan berhenti aman pada $d_{\text{stop}} = 0.546\text{ m}$.
      * Untuk rintangan besar seperti **Bus** ($K_{multiplier} = 1.5$, ukuran elips horizontal besar $a=1.2\text{ m}$), robot mulai bereaksi lebih awal pada $d_{\text{start}} = 1.820\text{ m}$ dengan puncak gaya repulsif pengereman yang lebih tinggi ($F_{rep, max} = 263.42\text{ N}$) dan berhenti aman pada jarak $d_{\text{stop}} = 0.636\text{ m}$.
    * **Analisis Pengaruh Parameter K_multiplier (Studi Kasus Truk)**:
      Eksperimen komparasi Truk dengan variasi $K_{multiplier}$ ($1.5$ dan $2.0$) menunjukkan peran kritis dari parameter pengali gaya ini:
      * **Jarak Reaksi Pengereman ($d_{\text{start}}$)**: Meningkatkan $K_{multiplier}$ dari $1.5$ ke $2.0$ memaksa sistem untuk mulai mengerem lebih dini, yaitu dari **1.826 m** menjadi **1.894 m** (lebih cepat $\approx 6.8\text{ cm}$). Hal ini sangat penting untuk meredam inersia kendaraan besar.
      * **Gaya Deselerasi ($F_{rep, max}$)**: Gaya repulsif maksimum meningkat drastis dari **399.43 N** ke **484.20 N** (kenaikan sebesar $21.2\%$). Kurva gaya deselerasi pada grafik komparasi menunjukkan laju kenaikan yang jauh lebih curam (*slope* tinggi), menandakan pengereman darurat yang lebih responsif dan agresif.
      * **Margin Jarak Berhenti ($d_{\text{stop}}$)**: Peningkatan gaya repulsif yang lebih agresif menyebabkan kendaraan berhenti lebih cepat dan menyisakan margin jarak aman yang lebih jauh dari bumper depan ke objek, yaitu dari **0.520 m** menjadi **0.548 m** (lebih jauh $2.8\text{ cm}$). Hal ini membuktikan bahwa penyetelan parameter $K_{multiplier}$ yang kontekstual sangat efektif dalam meningkatkan faktor keselamatan sistem ACC otonom.

3. **Analisis Eksperimen 3 (Sifat Vektor Terpadu Coupled LKA-SFM)**:
   * Menganalisis interaksi gaya lateral dalam Coupled SFM:
     $$F_{\text{total}, y} = F_{\text{lane}, y} + F_{\text{obs}, y}$$
   * Ketika rintangan berada di sisi kiri ($F_{\text{obs}, y} > 0$), gaya ini mendorong robot ke kanan. Di saat yang sama, LKA akan menghasilkan gaya penahan ke kiri ($F_{\text{lane}, y} < 0$) ketika robot mulai condong mendekati marka kanan.
   * Analisis batas kemampuan sistem: Jika objek memotong hingga $30\text{ cm}$, gaya tolak rintangan $F_{\text{obs}, y}$ mendominasi dan memaksa roda membelok tajam ke kanan hingga melewati batas marka jalan kanan karena lajur yang tersisa terlalu sempit untuk dilewati oleh dimensi robot Ackerman.

4. **Analisis Eksperimen 4 (Trade-off Latensi Filter vs Kestabilan Kontrol)**:
   * Menganalisis sifat filter Exponential Moving Average (EMA) pada LKA dengan Deadzone 30 piksel. Terdapat hubungan terbalik (*trade-off*) yang jelas antara tingkat kehalusan kemudi dan waktu respons kemudi.
   * Pada `alpha_filter = 1.0` (tanpa filter), setir merespon secara instan (delay $0\text{ ms}$), namun menghasilkan gerakan setir yang sangat kasar/getar dengan frekuensi koreksi tinggi sebesar **21.3 - 27.4 cycles/min** (pada $v = 0.5\text{ m/s}$).
   * Penggunaan filter yang lebih kuat secara bertahap menghaluskan gerakan setir. Pada `alpha_filter = 0.6`, respon delay mulai terasa sebesar **~70 ms** namun osilasi kemudi menjadi sangat minim dan stabil.
   * Pada nilai default `alpha_filter = 0.2`, gerakan setir menjadi sangat mulus dengan redaman osilasi optimal, namun terdapat response delay akumulatif sekitar **~180 ms** yang merupakan batas aman toleransi sebelum mobil mengalami *understeer* di tikungan tajam.

5. **Analisis Parameter Jarak Pandang Depan LKA (Pemilihan `Y_target` / `center_y` = 300 piksel)**:
   * Menganalisis alasan mengapa nilai target $y = 300$ piksel dipilih sebagai nilai paling optimal untuk pengoperasian robot:
     * **Visibilitas Marka Jalan**: Pada koordinat vertikal $y = 300$, bidang pandang kamera masih cukup lebar untuk mendeteksi marka kiri dan kanan secara bersamaan (tidak terlalu menyempit seperti pada perspektif jauh di $y < 200$). Hal ini memberikan akurasi penentuan titik tengah lajur yang tinggi dan stabil.
     * **Optimasi Waktu Respon Belok**: Jarak pandang depan di $y = 300$ memberikan jarak reaksi fisik yang seimbang ($\approx 50\text{ cm}$ di depan bumper robot). Jika target diletakkan terlalu jauh (misal $y = 200$), mobil otonom akan mulai melakukan manuver belok terlalu dini (prematur) sebelum benar-benar memasuki mulut tikungan. Sebaliknya, jika target diletakkan terlalu dekat (misal $y = 400$), mobil otonom akan telat berbelok dan mengalami *understeer* (keluar marka).
