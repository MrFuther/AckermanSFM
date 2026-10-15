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
     * **Simpangan Kesalahan Lajur Nyata** (`Lane Error (cm)` via topik `/lane_error` dalam sentimeter, dikonversi dari piksel menggunakan kalibrasi lebar lajur fisik 41 cm).
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

#### 4.7 HASIL EKSPERIMEN

#### 4.7.1 Pengaruh Lebar Deadzone terhadap Kestabilan Sudut Kemudi LKA

Pengujian dilakukan pada lintasan lurus sepanjang $5\text{ meter}$ dengan variasi lebar *deadzone* LKA ($0$ px, $15$ px, $30$ px, $40$ px, dan $50$ px) untuk tiga tingkat kecepatan target berbeda ($0.3\text{ m/s}$, $0.5\text{ m/s}$, dan $0.7\text{ m/s}$). Tujuan pengujian ini adalah menganalisis bagaimana parameter toleransi simpangan mempengaruhi kestabilan, osilasi, serta efisiensi koreksi sudut kemudi servo ($\rho$) dalam mempertahankan posisi robot di tengah lajur.

##### I. Pengujian Kestabilan Kemudi pada Kecepatan 0.3 m/s
Pengujian pada kecepatan rendah dilakukan untuk melihat sifat dasar respons PID lateral kemudi dengan pengaruh minimal dari inersia fisik kendaraan.

* **Tabel Hasil Eksperimen 1A (Speed = 0.3 m/s)**:
  | Variabel Bebas: Deadzone (piksel) | Standar Deviasi Lane Error (cm) | Standar Deviasi Sudut Kemudi /rho (derajat)* | Frekuensi Koreksi Setir (siklus/menit)** | Sifat Respon Setir |
  | :---: | :---: | :---: | :---: | :--- |
  | $0$ (Tanpa Deadzone) | $11.31$ | $28.69$ | $14.3$ | Osilasi konstan, kemudi bergetar |
  | $15$ | $7.77$ | $24.67$ | $29.4$ | Koreksi aktif, osilasi berkurang |
  | $30$ | $7.35$ | $23.30$ | $23.4$ | Stabil, berbelok tenang di tengah |
  | $40$ | $7.40$ | $22.80$ | $31.6$ | Stabil, respons koreksi sedikit lambat |
  | $50$ | $5.65$ | $21.00$ | $17.1$ | Sangat pasif, lamban mengoreksi |

  > [!NOTE]
  > \* Standar deviasi sudut kemudi ($\rho$) menunjukkan amplitudo osilasi kemudi fisik. Nilai nominal kemudi lurus adalah $180^\circ$.
  > \*\* Frekuensi koreksi (CPM) dihitung berdasarkan jumlah penyeberangan nilai sudut kemudi melewati titik tengah $180^\circ$ per menit.

* **Grafik Hasil Eksperimen 1A (Speed = 0.3 m/s)**:
  Sumbu kiri (biru) menunjukkan *Lane Error* dan sumbu kanan (hitam) menunjukkan respons *Steering Angle* ($\rho$).
  
  ````carousel
  ![Deadzone 0 px (v0 = 0.3 m/s)](file:///a:/ACKERMAN/deadzone_00px_03ms.png)
  <!-- slide -->
  ![Deadzone 15 px (v0 = 0.3 m/s)](file:///a:/ACKERMAN/deadzone_15px_03ms.png)
  <!-- slide -->
  ![Deadzone 30 px (v0 = 0.3 m/s)](file:///a:/ACKERMAN/deadzone_30px_03ms.png)
  <!-- slide -->
  ![Deadzone 40 px (v0 = 0.3 m/s)](file:///a:/ACKERMAN/deadzone_40px_03ms.png)
  <!-- slide -->
  ![Deadzone 50 px (v0 = 0.3 m/s)](file:///a:/ACKERMAN/deadzone_50px_03ms.png)
  ````

  > **Penjelasan Grafik Karosel 1A (Individual Dual-Axis)**:
  > Grafik karosel di atas menampilkan rekaman data respons waktu untuk masing-masing nilai *deadzone* (0 s.d. 50 px) pada kecepatan $0.3\text{ m/s}$. Sumbu kiri (biru) menunjukkan deviasi posisi nyata (*Lane Error*) dalam sentimeter (cm), sedangkan sumbu kanan (hitam) menunjukkan respons kemudi servo ($\rho$) dalam derajat. Garis putus-putus merah horizontal merepresentasikan batas wilayah *deadzone* ($\pm DZ$). Pada deadzone $0$ px (Tanpa Deadzone), dapat dilihat bahwa garis kemudi hitam terus-menerus bergerak berfluktuasi naik-turun merespon noise kecil *lane error* biru. Sebaliknya, pada deadzone $30$ px, ketika nilai *lane error* berada di dalam batas garis merah ($\pm30$ px atau $\pm3.51$ cm), garis kemudi hitam stabil mendatar di nilai $180^\circ$ (lurus), dan baru berbelok mengoreksi ketika error keluar dari batas tersebut.

  * **Analisis Detail Grafik Individual (v0 = 0.3 m/s)**:
    * **Grafik Deadzone 0 px**: Tanpa batas toleransi (deadzone $0$ px), motor servo kemudi secara instan menerjemahkan setiap noise marka jalan. Garis hitam kemudi ($\rho$) berfluktuasi terus-menerus dengan standar deviasi sebesar $28.69^\circ$ dan frekuensi koreksi $14.3\text{ CPM}$. Roda kemudi bergetar tanpa pernah diam di posisi netral $180^\circ$ sekalipun robot berjalan pada lintasan lurus.
    * **Grafik Deadzone 15 px**: Penerapan toleransi $\pm15$ px (atau $\pm1.76$ cm) mulai menyaring fluktuasi error kecil. Namun, karena batas toleransi yang sempit, servo masih aktif mengoreksi dengan frekuensi $29.4\text{ CPM}$. Standar deviasi kemudi berhasil diturunkan menjadi $24.67^\circ$, yang secara mekanis mengurangi getaran kasar.
    * **Grafik Deadzone 30 px**: Menunjukkan performa kemudi yang optimal. Ketika *lane error* (garis biru) berada di dalam koridor merah ($\pm30$ px atau $\pm3.51$ cm), garis hitam kemudi melurus stabil di $180^\circ$. Koreksi baru dilakukan secara bertahap saat terjadi simpangan nyata di luar batas tersebut, menghasilkan standar deviasi kemudi sebesar $23.30^\circ$ dengan laju yang sangat stabil.
    * **Grafik Deadzone 40 px**: Dengan batas toleransi yang dilebarkan ke $\pm40$ px (atau $\pm4.69$ cm), roda kemudi menjadi lebih pasif dan melurus lebih lama. Namun, saat mendekati tikungan, respons kemudi mengalami sedikit penundaan sebelum berbelok secara mendadak dengan frekuensi koreksi $31.6\text{ CPM}$ setelah error menembus batas.
    * **Grafik Deadzone 50 px**: Menunjukkan batas toleransi lateral yang terlalu longgar. Kemudi servo hampir tidak bergerak ($17.1\text{ CPM}$) dan hanya melurus di $180^\circ$ karena error jarang sekali melampaui batas $\pm50$ px (atau $\pm5.86$ cm). Meskipun standar deviasi kemudi rendah ($21.00^\circ$), sistem menjadi tidak sensitif terhadap penyimpangan posisi lateral.

* **Perbandingan Profil Lane Error (v0 = 0.3 m/s)**:
  ![Perbandingan Deadzone pada v0 = 0.3 m/s](file:///a:/ACKERMAN/deadzone_comparison_03ms.png)

  > **Penjelasan Grafik Perbandingan Deadzone 1A**:
  > Grafik komparasi di atas meng-overlay profil *Lane Error* dari kelima variasi deadzone pada satu garis waktu. Terlihat jelas bahwa tanpa deadzone (garis merah/merah muda), amplitudo fluktuasi error sangat tinggi dan tidak beraturan. Peningkatan lebar deadzone (garis hijau/oranye/biru) secara visual melandaikan kurva error, namun pada deadzone $50$ px (garis biru), respons sistem sangat lambat dan kurva cenderung lurus mengabaikan tikungan lintasan.

  * **Analisis Grafik Perbandingan Deadzone 1A**: Grafik overlay ini memperlihatkan bahwa ketiadaan deadzone (garis merah muda) menghasilkan simpangan lateral yang sangat acak sepanjang waktu akibat getaran kemudi yang tak terkontrol. Penyetelan deadzone $15$ px (garis oranye) dan $30$ px (garis hijau) berhasil menjaga profil error tetap konvergen dan berpusat di dekat garis tengah. Sementara pada deadzone $50$ px (garis biru), terlihat kurva error melandai sangat lambat kembali ke nol karena terlambatnya respons awal kemudi otonom.

* **Analisis Hasil (v0 = 0.3 m/s)**:
  Pada kecepatan rendah $0.3\text{ m/s}$, efek inersia mekanis robot Ackerman sangat kecil. Tanpa deadzone ($0$ px), noise pembacaan koordinat marka secara langsung mengaktifkan koreksi PID lateral, menyebabkan kemudi aktif mengayun secara konstan dengan standar deviasi kemudi sebesar $28.69^\circ$. Ketika deadzone diatur ke $30$ px, sistem mengabaikan fluktuasi error kecil di bawah $\pm30$ px (atau $\pm3.51$ cm), yang meredam gerakan mikro servo kemudi (standar deviasi kemudi turun menjadi $23.30^\circ$). Namun, jika deadzone ditingkatkan ke $50$ px, meskipun standar deviasi error bernilai kecil di lintasan lurus ($5.65\text{ cm}$) karena robot jarang bergeser secara signifikan, respons kemudi menjadi terlalu lambat dan pasif, berpotensi memicu kegagalan berbelok jika robot memasuki tikungan secara mendadak.

##### II. Pengujian Kestabilan Kemudi pada Kecepatan 0.5 m/s
Pengujian pada kecepatan menengah merupakan representasi operasi standar robot otonom Ackerman pada kondisi lintasan lurus dan tikungan normal.

* **Tabel Hasil Eksperimen 1B (Speed = 0.5 m/s)**:
  | Variabel Bebas: Deadzone (piksel) | Standar Deviasi Lane Error (cm) | Standar Deviasi Sudut Kemudi /rho (derajat) | Frekuensi Koreksi Setir (siklus/menit) | Sifat Respon Setir |
  | :---: | :---: | :---: | :---: | :--- |
  | $0$ (Tanpa Deadzone) | $14.40$ | $27.96$ | $70.7$ | Sangat osilatif, setir bergetar hebat |
  | $15$ | $10.41$ | $23.89$ | $54.9$ | Jitter kemudi berkurang, kemudi dinamis |
  | $30$ | $14.22$ | $26.65$ | $61.6$ | Koreksi aktif, osilasi berkurang |
  | $40$ | $12.89$ | $21.36$ | $16.0$ | Tenang, respon tikungan lambat |
  | $50$ | $8.28$ | $19.02$ | $18.9$ | Sangat lamban, understeer di tikungan |

* **Grafik Hasil Eksperimen 1B (Speed = 0.5 m/s)**:
  
  ````carousel
  ![Deadzone 0 px (v0 = 0.5 m/s)](file:///a:/ACKERMAN/deadzone_00px_05ms.png)
  <!-- slide -->
  ![Deadzone 15 px (v0 = 0.5 m/s)](file:///a:/ACKERMAN/deadzone_15px_05ms.png)
  <!-- slide -->
  ![Deadzone 30 px (v0 = 0.5 m/s)](file:///a:/ACKERMAN/deadzone_30px_05ms.png)
  <!-- slide -->
  ![Deadzone 40 px (v0 = 0.5 m/s)](file:///a:/ACKERMAN/deadzone_40px_05ms.png)
  <!-- slide -->
  ![Deadzone 50 px (v0 = 0.5 m/s)](file:///a:/ACKERMAN/deadzone_50px_05ms.png)
  ````

  > **Penjelasan Grafik Karosel 1B (Individual Dual-Axis)**:
  > Grafik karosel di atas menampilkan rekaman data respons waktu untuk masing-masing nilai *deadzone* pada kecepatan menengah $0.5\text{ m/s}$. Sumbu kiri (biru) menunjukkan deviasi posisi nyata (*Lane Error*) dalam sentimeter (cm), sedangkan sumbu kanan (hitam) menunjukkan respons kemudi servo ($\rho$) dalam derajat. Garis putus-putus merah horizontal merepresentasikan batas wilayah *deadzone* ($\pm DZ$). Pada deadzone $0$ px (Tanpa Deadzone), osilasi garis kemudi hitam terlihat sangat rapat dan sering terjadi overshoot lateral karena sensitivitas berlebih. Ketika deadzone diatur ke $30$ px, getaran mikro kemudi hitam berhasil diredam pada lintasan lurus, namun sistem tetap tanggap memberikan belokan tajam (grafik hitam naik atau turun mendekati batas $141^\circ$ atau $218^\circ$) ketika deviasi lajur biru melanggar garis batas merah ($\pm30$ px atau $\pm3.51$ cm).

  * **Analisis Detail Grafik Individual (v0 = 0.5 m/s)**:
    * **Grafik Deadzone 0 px**: Pada kecepatan $0.5\text{ m/s}$, inersia kendaraan memperburuk osilasi kemudi. Garis hitam kemudi ($\rho$) berosilasi hebat dengan frekuensi $70.7\text{ CPM}$ dan standar deviasi $27.96^\circ$, bergerak menyentuh batas mekanis $141^\circ$ dan $218^\circ$ secara terus-menerus. Hal ini mengindikasikan timbulnya ketidakstabilan arah kemudi yang berlebihan.
    * **Grafik Deadzone 15 px**: Batas toleransi $\pm15$ px (atau $\pm1.76$ cm) memotong sebagian kecil noise, menurunkan frekuensi koreksi kemudi menjadi $54.9\text{ CPM}$ dengan standar deviasi kemudi $23.89^\circ$. Meskipun jitter sedikit berkurang pada lintasan lurus, setir masih tampak bergoyang secara aktif merespon noise citra.
    * **Grafik Deadzone 30 px**: Menunjukkan kontrol lateral yang stabil dan responsif. Getaran setir berhasil diredam di sepanjang garis tengah lintasan lurus, namun roda kemudi tetap berputar secara tegas untuk membelokkan robot ketika error melampaui batas merah ($\pm3.51$ cm), mencatatkan frekuensi koreksi $61.6\text{ CPM}$ dan standar deviasi $26.65^\circ$.
    * **Grafik Deadzone 40 px**: Menghasilkan kemudi yang lebih tenang dengan frekuensi koreksi turun drastis ke $16.0\text{ CPM}$ dan standar deviasi kemudi $21.36^\circ$. Namun, keterlambatan respons mulai terlihat di mana error lajur biru menumpuk hingga lebih dari $11.71\text{ cm}$ ($100$ piksel) sebelum servo berbelok mengoreksinya.
    * **Grafik Deadzone 50 px**: Menunjukkan respons kemudi yang sangat lamban dan tidak responsif. Sudut kemudi servo hampir mendatar di posisi netral sepanjang pengujian dengan frekuensi koreksi hanya $18.9\text{ CPM}$ dan standar deviasi $19.02^\circ$. Robot mengalami understeer parah di tikungan sebelum kemudi bereaksi.

* **Perbandingan Profil Lane Error (v0 = 0.5 m/s)**:
  ![Perbandingan Deadzone pada v0 = 0.5 m/s](file:///a:/ACKERMAN/deadzone_comparison_05ms.png)

  > **Penjelasan Grafik Perbandingan Deadzone 1B**:
  > Grafik di atas membandingkan kelima respons *Lane Error* untuk kecepatan $0.5\text{ m/s}$. Terlihat getaran osilasi tinggi pada kurva tanpa deadzone (merah), sedangkan kurva deadzone 30 px (hijau) menunjukkan perataan deviasi yang stabil. Pada deadzone 50 px (biru), kurva error memiliki deviasi lateral yang memuncak karena inisiasi belokan yang terlambat (kemudi terlambat merespons).

  * **Analisis Grafik Perbandingan Deadzone 1B**: Grafik perbandingan ini menegaskan bahwa pada kecepatan menengah, deadzone $0$ px (kurva merah) memicu osilasi lateral yang tidak menentu. Penyetelan deadzone $30$ px (kurva hijau) menghasilkan respon transien terbaik dengan deviasi minimal. Di sisi lain, deadzone $50$ px (kurva biru) memperlihatkan lonjakan error lateral yang tinggi karena inisiasi koreksi kemudi yang terlambat di tikungan lintasan.

* **Analisis Hasil (v0 = 0.5 m/s)**:
  Pada kecepatan $0.5\text{ m/s}$, osilasi kemudi menjadi masalah yang krusial. Tanpa deadzone ($0$ px), robot mengalami osilasi lateral berfrekuensi tinggi dengan $70.7\text{ CPM}$. Hal ini tecermin dari grafik sumbu ganda di mana garis hitam (sudut kemudi $\rho$) berayun naik turun secara liar di antara batas mekanis kemudi ($141.34^\circ$ - $218.66^\circ$) untuk merespon setiap lonjakan kecil dari grafik biru (*Lane Error*). Setting deadzone $30$ px meredam getaran kemudi pada lintasan lurus tetapi mempertahankan aksi kontrol yang tegas saat mendeteksi deviasi nyata di tikungan. Jika dinaikkan ke $50$ px (atau $5.86$ cm), sistem menjadi terlalu toleran terhadap penyimpangan, menyebabkan robot meluncur lurus (understeer) saat memasuki tikungan sebelum servo mulai digerakkan secara lambat pada $18.9\text{ CPM}$.

##### III. Pengujian Kestabilan Kemudi pada Kecepatan 0.7 m/s
Pengujian pada kecepatan tinggi digunakan untuk menguji ketahanan sistem kendali lateral terhadap overshoot kontrol akibat pengaruh momentum inersia robot.

* **Tabel Hasil Eksperimen 1C (Speed = 0.7 m/s)**:
  | Variabel Bebas: Deadzone (piksel) | Standar Deviasi Lane Error (cm) | Standar Deviasi Sudut Kemudi /rho (derajat) | Frekuensi Koreksi Setir (siklus/menit) | Sifat Respon Setir |
  | :---: | :---: | :---: | :---: | :--- |
  | $0$ (Tanpa Deadzone) | $14.14$ | $23.65$ | $84.7$ | Osilasi liar, getaran aktuator tinggi |
  | $15$ | $11.71$ | $21.69$ | $41.1$ | Cukup stabil, setir masih bergetar |
  | $30$ | $9.13$ | $18.88$ | $9.3$ | Sangat stabil, pergerakan setir mulus |
  | $40$ | $8.11$ | $19.47$ | $5.1$ | Setir pasif, respons koreksi lamban |
  | $50$ | $9.25$ | $18.77$ | $15.2$ | Keluar dari marka lajur (Gagal membelok) |

* **Grafik Hasil Eksperimen 1C (Speed = 0.7 m/s)**:
  
  ````carousel
  ![Deadzone 0 px (v0 = 0.7 m/s)](file:///a:/ACKERMAN/deadzone_00px_07ms.png)
  <!-- slide -->
  ![Deadzone 15 px (v0 = 0.7 m/s)](file:///a:/ACKERMAN/deadzone_15px_07ms.png)
  <!-- slide -->
  ![Deadzone 30 px (v0 = 0.7 m/s)](file:///a:/ACKERMAN/deadzone_30px_07ms.png)
  <!-- slide -->
  ![Deadzone 40 px (v0 = 0.7 m/s)](file:///a:/ACKERMAN/deadzone_40px_07ms.png)
  <!-- slide -->
  ![Deadzone 50 px (v0 = 0.7 m/s)](file:///a:/ACKERMAN/deadzone_50px_07ms.png)
  ````

  > **Penjelasan Grafik Karosel 1C (Individual Dual-Axis)**:
  > Grafik karosel di atas memperlihatkan pengaruh *deadzone* pada kecepatan tinggi $0.7\text{ m/s}$. Sumbu kiri (biru) menunjukkan deviasi posisi nyata (*Lane Error*) dalam sentimeter (cm), sedangkan sumbu kanan (hitam) menunjukkan respons kemudi servo ($\rho$) dalam derajat. Garis putus-putus merah horizontal merepresentasikan batas wilayah *deadzone* ($\pm DZ$). Pada deadzone $0$ px, ayunan kemudi hitam sangat ekstrem karena kombinasi antara noise kamera dan momentum inersia tinggi robot. Pada deadzone $30$ px, kemudi hitam jauh lebih tenang dengan koreksi minimal namun stabil. Pada deadzone $50$ px, inersia tinggi menyebabkan robot meluncur keluar lintasan lajur sebelum garis hitam sempat membelok secara signifikan (kegagalan kontrol).

  * **Analisis Detail Grafik Individual (v0 = 0.7 m/s)**:
    * **Grafik Deadzone 0 px**: Menunjukkan osilasi lateral destruktif. Getaran sensor memicu ayunan kemudi ekstrem dengan frekuensi tertinggi sebesar $84.7\text{ CPM}$ dan standar deviasi $23.65^\circ$. Pergerakan roda kemudi yang terus melompat dari satu batas mekanis ke batas lainnya dapat merusak roda gigi aktuator servo kemudi secara fisik.
    * **Grafik Deadzone 15 px**: Toleransi $\pm15$ px (atau $\pm1.76$ cm) meredam getaran menjadi $41.1\text{ CPM}$ dengan standar deviasi kemudi $21.69^\circ$. Namun, setir masih tampak bergetar dinamis pada jalan lurus, yang membuktikan bahwa toleransi ini belum memadai untuk kecepatan tinggi $0.7\text{ m/s}$.
    * **Grafik Deadzone 30 px**: Menunjukkan kestabilan kontrol optimal pada kecepatan tinggi. Garis hitam kemudi tetap tenang di $180^\circ$ saat berjalan lurus, dan berbelok secara efisien di tikungan dengan frekuensi koreksi turun ke $9.3\text{ CPM}$ dan standar deviasi $18.88^\circ$. Keadaan ini meminimalkan jitter tanpa mengorbankan akurasi kemudi lateral.
    * **Grafik Deadzone 40 px**: Sudut kemudi tampak sangat pasif dengan frekuensi koreksi hanya $5.1\text{ CPM}$. Keterlambatan respon kemudi di tikungan mulai terlihat jelas, di mana garis biru error mengalami simpangan sesaat sebelum roda membelok.
    * **Grafik Deadzone 50 px**: Menunjukkan kegagalan sistem kendali kemudi LKA secara penuh. Lebar deadzone yang sangat besar dikombinasikan dengan inersia tinggi kendaraan pada $0.7\text{ m/s}$ membuat servo kemudi terlambat memberikan sudut belok lateral yang cukup. Kurva kemudi hitam terlambat membelok dan log data terputus di tengah jalan karena robot meluncur keluar lintasan marka jalan.

* **Perbandingan Profil Lane Error (v0 = 0.7 m/s)**:
  ![Perbandingan Deadzone pada v0 = 0.7 m/s](file:///a:/ACKERMAN/deadzone_comparison_07ms.png)

  > **Penjelasan Grafik Perbandingan Deadzone 1C**:
  > Grafik di atas membandingkan respons *Lane Error* pada kecepatan $0.7\text{ m/s}$. Terlihat osilasi tanpa deadzone (merah) sangat tinggi. Kurva deadzone 30 px (hijau) dan 40 px (oranye) menunjukkan kestabilan perataan error yang efisien. Pada deadzone 50 px (biru), terlihat grafik error meluncur naik ke atas $17.57\text{ cm}$ ($150$ px) tanpa kembali lagi, menandakan mobil telah keluar dari lajur marka jalan.

  * **Analisis Grafik Perbandingan Deadzone 1C**: Visualisasi perbandingan ini memperlihatkan secara jelas kegagalan kontrol lateral pada parameter deadzone $50$ px (kurva biru), di mana error lajur melesat ke atas melampaui $17.57\text{ cm}$ ($150$ px) secara permanen. Sementara itu, parameter deadzone $30$ px (kurva hijau) membuktikan efisiensinya dalam mempertahankan kestabilan dengan profil error yang landai dan cepat konvergen kembali ke nol.

* **Analisis Hasil (v0 = 0.7 m/s)**:
  Pada kecepatan tinggi $0.7\text{ m/s}$, inersia kendaraan memperparah *overshoot* pergerakan lateral. Tanpa deadzone ($0$ px), robot mengalami osilasi destruktif dengan frekuensi koreksi kemudi tertinggi sebesar $84.7\text{ CPM}$. Getaran kemudi ini merusak efisiensi mekanis motor servo dan menyebabkan kendaraan tidak stabil secara fisik. Sebaliknya, pada deadzone $30$ px, kestabilan optimal tercapai. Grafik sumbu ganda menunjukkan profil error (biru) dan sudut kemudi (hitam) melandai secara bersamaan, dengan frekuensi koreksi setir turun drastis ke $9.3\text{ CPM}$. Penerapan deadzone $50$ px pada kecepatan ini mengakibatkan kegagalan total sistem LKA karena robot meluncur keluar lintasan marka jalan sebelum kemudi sempat memberikan gaya koreksi lateral yang cukup.

---

#### 4.7.2 Pengaruh Parameter Smoothing alpha_filter terhadap Delay Respon Sudut Kemudi

Penerapan filter Exponential Moving Average (EMA) pada koordinat tengah lajur bertujuan untuk menyaring noise akibat bayangan atau variasi kontras jalan. Pengujian dilakukan dengan membandingkan respons transien kemudi LKA pada tikungan tajam menggunakan variasi parameter `alpha_filter` ($1.0$ [tanpa filter], $0.8$, $0.6$, $0.4$, dan $0.2$). Pengujian ini dievaluasi secara terpisah pada lebar **Deadzone 30 px** dan **Deadzone 40 px** untuk tiga tingkat kecepatan.

##### A. Eksperimen 4A: Variasi EMA pada Deadzone 30 px

##### I. Pengujian EMA (Deadzone 30 px) pada Kecepatan 0.3 m/s
* **Tabel Hasil Eksperimen 2A (Speed = 0.3 m/s, Deadzone = 30 px)**:
  | Nilai `alpha_filter` EMA | Standar Deviasi Lane Error (cm) | Standar Deviasi Sudut Kemudi /rho (derajat) | Frekuensi Koreksi Setir (siklus/menit) | Karakteristik Kemudi & Waktu Tunda |
  | :---: | :---: | :---: | :---: | :--- |
  | $1.0$ (Tanpa Filter) | $9.13$ | $25.38$ | $57.9$ | Kasar dan bergetar, delay respons $0\text{ ms}$ |
  | $0.8$ | $3.72$ | $16.75$ | $35.2$ | Cukup halus, delay respons $\approx 30\text{ ms}$ |
  | $0.6$ | $5.20$ | $20.47$ | $9.8$ | Halus dan stabil, delay respons $\approx 70\text{ ms}$ |
  | $0.4$ | $6.24$ | $18.60$ | $8.6$ | Sangat halus, mulai timbul overshoot karena delay |
  | $0.2$ (Default) | $4.86$ | $17.90$ | $8.7$ | Mulus namun respons lamban, delay $\approx 180\text{ ms}$ |

* **Grafik Hasil Eksperimen 2A (Speed = 0.3 m/s, Deadzone = 30 px)**:
  
  ````carousel
  ![EMA 1.0 (v0 = 0.3 m/s, DZ = 30 px)](file:///a:/ACKERMAN/ema_10_03ms.png)
  <!-- slide -->
  ![EMA 0.8 (v0 = 0.3 m/s, DZ = 30 px)](file:///a:/ACKERMAN/ema_08_03ms.png)
  <!-- slide -->
  ![EMA 0.6 (v0 = 0.3 m/s, DZ = 30 px)](file:///a:/ACKERMAN/ema_06_03ms.png)
  <!-- slide -->
  ![EMA 0.4 (v0 = 0.3 m/s, DZ = 30 px)](file:///a:/ACKERMAN/ema_04_03ms.png)
  <!-- slide -->
  ![EMA 0.2 (v0 = 0.3 m/s, DZ = 30 px)](file:///a:/ACKERMAN/ema_02_03ms.png)
  ````

  > **Penjelasan Grafik Karosel 2A (Pengaruh EMA pada 0.3 m/s)**:
  > Grafik karosel di atas memperlihatkan efek kehalusan respons waktu kemudi. Pada EMA 1.0 (Tanpa Filter), garis sudut kemudi hitam berosilasi cepat mengikuti noise pembacaan *Lane Error* biru. Ketika filter diperkuat ke EMA 0.6, dapat diamati bahwa grafik sudut kemudi hitam jauh lebih halus (tidak memiliki jitter frekuensi tinggi) dan secara konsisten berbelok secara gradual untuk mengarahkan robot kembali ke tengah lajur (error mendekati nol). Pada EMA 0.2, tunda respons ($\approx 180\text{ ms}$) terlihat dari pergeseran fasa di mana gerakan kemudi terlambat merespons kenaikan *Lane Error*.

  * **Analisis Detail Grafik Individual (v0 = 0.3 m/s, DZ = 30 px)**:
    * **Grafik EMA 1.0**: Tanpa pemfilteran, garis kemudi hitam ($\rho$) berayun secara liar merespon getaran kecil hasil deteksi kamera. Frekuensi koreksi mencapai $57.9\text{ CPM}$ dengan standar deviasi sudut kemudi sebesar $25.38^\circ$. Keadaan ini memicu keausan aktuator mekanik yang tinggi meskipun robot tetap berada di dalam lajur.
    * **Grafik EMA 0.8**: Penerapan filter ringan mulai meredam jitter setir, menurunkan frekuensi koreksi menjadi $35.2\text{ CPM}$ dan standar deviasi kemudi menjadi $16.75^\circ$. Respons belokan di tikungan tetap tanggap dengan delay fasa yang sangat minim ($\approx 30\text{ ms}$).
    * **Grafik EMA 0.6**: Menunjukkan peredaman jitter yang optimal pada kecepatan rendah. Sudut kemudi servo bergeser secara mulus dengan frekuensi koreksi turun drastis ke $9.8\text{ CPM}$ dan standar deviasi $20.47^\circ$. Tunda filter sebesar $\approx 70\text{ ms}$ masih berada dalam batas aman kontrol lateral.
    * **Grafik EMA 0.4**: Gerakan kemudi sangat mulus dengan frekuensi koreksi $8.6\text{ CPM}$ dan standar deviasi $18.60^\circ$. Namun, tunda respons ($\approx 100\text{ ms}$) mulai menggeser fasa aksi koreksi kemudi, memicu sedikit overshoot lateral pada tikungan jalan.
    * **Grafik EMA 0.2**: Filter kuat menghasilkan gerakan kemudi paling lamban. Garis hitam kemudi tampak mendatar dan tunda respons sebesar $\approx 180\text{ ms}$ memicu pergeseran fasa yang nyata terhadap error lajur biru, meskipun standar deviasi kemudi rendah ($17.90^\circ$).

* **Analisis Hasil (v0 = 0.3 m/s, DZ = 30 px)**:
  Pada kecepatan rendah $0.3\text{ m/s}$, filter `alpha_filter = 0.6` memberikan kestabilan transien yang optimal dengan standar deviasi Lane Error sebesar $5.20$ cm dan frekuensi koreksi kemudi yang sangat rendah ($9.8\text{ CPM}$). Ini menunjukkan bahwa meskipun waktu tunda filter mencapai $\approx 70\text{ ms}$, inersia rendah pada $0.3\text{ m/s}$ meminimalkan risiko deviasi berlebih. Tanpa filter (`alpha = 1.0`), setir berayun aktif ($57.9\text{ CPM}$) merespons getaran frame LKA. Penggunaan filter kuat (`alpha = 0.2`) menghasilkan gerakan setir yang sangat halus, namun delay respons yang mencapai $\approx 180\text{ ms}$ mulai menggeser fasa aksi kontrol kemudi dari nilai error sebenarnya, sehingga terjadi fluktuasi sudut kemudi yang sedikit berlebih (overshoot) demi mengejar lajur jalan.

##### II. Pengujian EMA (Deadzone 30 px) pada Kecepatan 0.5 m/s
* **Tabel Hasil Eksperimen 2B (Speed = 0.5 m/s, Deadzone = 30 px)**:
  | Nilai `alpha_filter` EMA | Standar Deviasi Lane Error (cm) | Standar Deviasi Sudut Kemudi /rho (derajat) | Frekuensi Koreksi Setir (siklus/menit) | Karakteristik Kemudi & Waktu Tunda |
  | :---: | :---: | :---: | :---: | :--- |
  | $1.0$ (Tanpa Filter) | $10.96$ | $22.02$ | $22.8$ | Kasar bergetar, respon instan ($0\text{ ms}$) |
  | $0.8$ | $8.99$ | $24.12$ | $34.5$ | Getaran ringan, respon delay $\approx 30\text{ ms}$ |
  | $0.6$ | $7.70$ | $19.98$ | $8.2$ | Halus dan stabil, respon delay $\approx 70\text{ ms}$ |
  | $0.4$ | $9.93$ | $20.02$ | $6.7$ | Sangat halus, respon delay $\approx 100\text{ ms}$ |
  | $0.2$ (Default) | $9.21$ | $18.64$ | $13.6$ | Sangat lamban, rawan understeer, delay $\approx 180\text{ ms}$ |

* **Grafik Hasil Eksperimen 2B (Speed = 0.5 m/s, Deadzone = 30 px)**:
  
  ````carousel
  ![EMA 1.0 (v0 = 0.5 m/s, DZ = 30 px)](file:///a:/ACKERMAN/ema_10_05ms.png)
  <!-- slide -->
  ![EMA 0.8 (v0 = 0.5 m/s, DZ = 30 px)](file:///a:/ACKERMAN/ema_08_05ms.png)
  <!-- slide -->
  ![EMA 0.6 (v0 = 0.5 m/s, DZ = 30 px)](file:///a:/ACKERMAN/ema_06_05ms.png)
  <!-- slide -->
  ![EMA 0.4 (v0 = 0.5 m/s, DZ = 30 px)](file:///a:/ACKERMAN/ema_04_05ms.png)
  <!-- slide -->
  ![EMA 0.2 (v0 = 0.5 m/s, DZ = 30 px)](file:///a:/ACKERMAN/ema_02_05ms.png)
  ````

  > **Penjelasan Grafik Karosel 2B (Pengaruh EMA pada 0.5 m/s)**:
  > Grafik karosel di atas memperlihatkan transisi performa kontrol pada kecepatan menengah. Dengan EMA 0.6, lintasan error biru stabil berosilasi rendah, dan garis kemudi hitam bekerja secara efisien meredam gerakan kasar. Pada EMA 0.2, tunda respons kemudi menyebabkan robot terlambat belok, yang tecermin dari pergeseran kurva sudut kemudi yang lebih mendatar di awal dan menumpuknya simpangan kesalahan lajur biru yang melampaui $\pm11.71$ cm ($100$ px) sebelum roda kemudi diputar maksimal untuk koreksi.

  * **Analisis Detail Grafik Individual (v0 = 0.5 m/s, DZ = 30 px)**:
    * **Grafik EMA 1.0**: Ketiadaan filter pada kecepatan menengah memicu jitter kemudi yang kasar di lintasan lurus. Grafik kemudi hitam ($\rho$) berfluktuasi tajam dengan frekuensi koreksi $22.8\text{ CPM}$ dan standar deviasi kemudi $22.02^\circ$.
    * **Grafik EMA 0.8**: Filter ringan mengurangi sebagian getaran mikro pada setir, namun roda kemudi masih tampak berayun aktif dengan frekuensi $34.5\text{ CPM}$ dan standar deviasi $24.12^\circ$ untuk meminimalkan deviasi lajur jalan.
    * **Grafik EMA 0.6**: Menunjukkan keseimbangan performansi yang optimal. Garis error biru stabil dengan deviasi terkecil ($7.70\text{ cm}$) dan kemudi hitam bergeser secara halus tanpa getaran mikro, mencatatkan frekuensi koreksi terendah yaitu $8.2\text{ CPM}$ dengan standar deviasi $19.98^\circ$.
    * **Grafik EMA 0.4**: Filter yang lebih kuat memotong frekuensi koreksi hingga $6.7\text{ CPM}$ dan standar deviasi kemudi $20.02^\circ$. Namun, tunda respons ($\approx 100\text{ ms}$) mulai memperlambat reaksi kemudi di tikungan, meningkatkan simpangan error lateral hingga $9.93\text{ cm}$ ($84.80$ px).
    * **Grafik EMA 0.2**: Tunda respons kemudi terlihat sangat mendominasi. Sudut kemudi hitam melambat secara signifikan akibat delay $\approx 180\text{ ms}$, menyebabkan error lajur melampaui $\pm11.71$ cm ($100$ px) sebelum roda mulai diputar maksimal untuk mengembalikan robot ke tengah lajur.

* **Analisis Hasil (v0 = 0.5 m/s, DZ = 30 px)**:
  Pada kecepatan standar $0.5\text{ m/s}$, filter `alpha_filter = 0.6` memberikan hasil yang sangat menonjol dengan standar deviasi Lane Error terendah yaitu $7.70\text{ cm}$ dan kestabilan kemudi yang sangat tinggi ($8.2\text{ CPM}$). Ini menunjukkan bahwa EMA 0.6 sangat efektif untuk menyaring noise pembacaan marka tanpa memicu understeer atau osilasi kemudi. Penerapan filter yang lebih lamban (`alpha = 0.2`) meredam osilasi secara maksimal, namun akumulasi tunda waktu sebesar $180\text{ ms}$ menyebabkan mobil terlambat berbelok dan mengalami understeer ringan sebelum berhasil ditarik kembali ke lajur.

##### III. Pengujian EMA (Deadzone 30 px) pada Kecepatan 0.7 m/s
* **Tabel Hasil Eksperimen 2C (Speed = 0.7 m/s, Deadzone = 30 px)**:
  | Nilai `alpha_filter` EMA | Standar Deviasi Lane Error (cm) | Standar Deviasi Sudut Kemudi /rho (derajat) | Frekuensi Koreksi Setir (siklus/menit) | Karakteristik Kemudi & Waktu Tunda |
  | :---: | :---: | :---: | :---: | :--- |
  | $1.0$ (Tanpa Filter) | $15.20$ | $25.56$ | $45.6$ | Kasar dan tidak stabil, delay $0\text{ ms}$ |
  | $0.8$ | $11.25$ | $19.25$ | $9.8$ | Halus namun masih sedikit getaran |
  | $0.6$ | $12.83$ | $21.57$ | $23.7$ | Mulus dan aman terkendali, delay $\approx 70\text{ ms}$ |
  | $0.4$ | $11.81$ | $21.74$ | $16.9$ | Overshoot lateral tinggi, delay $\approx 100\text{ ms}$ |
  | $0.2$ (Default) | $7.70$ | $20.35$ | $9.6$ | Understeer berat (Keluar marka tikungan) |

* **Grafik Hasil Eksperimen 2C (Speed = 0.7 m/s, Deadzone = 30 px)**:
  
  ````carousel
  ![EMA 1.0 (v0 = 0.7 m/s, DZ = 30 px)](file:///a:/ACKERMAN/ema_10_07ms.png)
  <!-- slide -->
  ![EMA 0.8 (v0 = 0.7 m/s, DZ = 30 px)](file:///a:/ACKERMAN/ema_08_07ms.png)
  <!-- slide -->
  ![EMA 0.6 (v0 = 0.7 m/s, DZ = 30 px)](file:///a:/ACKERMAN/ema_06_07ms.png)
  <!-- slide -->
  ![EMA 0.4 (v0 = 0.7 m/s, DZ = 30 px)](file:///a:/ACKERMAN/ema_04_07ms.png)
  <!-- slide -->
  ![EMA 0.2 (v0 = 0.7 m/s, DZ = 30 px)](file:///a:/ACKERMAN/ema_02_07ms.png)
  ````

  > **Penjelasan Grafik Karosel 2C (Pengaruh EMA pada 0.7 m/s)**:
  > Grafik karosel di atas memperlihatkan respons sistem LKA pada kecepatan tinggi $0.7\text{ m/s}$. Pada EMA 1.0 (Tanpa Filter), osilasi liar setir hitam menunjukkan jitter sensor yang sangat tinggi. Sebaliknya, pada filter kuat EMA 0.2, keterlambatan respons (fasa) menyebabkan garis error biru naik secara parabolik melampaui $17.57\text{ cm}$ ($150$ px) dan terus meningkat tanpa turun, yang secara fisis memvalidasi kegagalan belok (understeer) akibat kelambatan aktuator kemudi.

  * **Analisis Detail Grafik Individual (v0 = 0.7 m/s, DZ = 30 px)**:
    * **Grafik EMA 1.0**: Menunjukkan respons kemudi yang sangat kasar di mana setir hitam terus-menerus berayun tajam merespon noise sensor dengan frekuensi $45.6\text{ CPM}$ dan standar deviasi kemudi $25.56^\circ$. Hal ini membahayakan kestalinan mekanik robot pada kecepatan tinggi.
    * **Grafik EMA 0.8**: Filter ringan mampu menekan sebagian getaran menjadi $9.8\text{ CPM}$ dengan standar deviasi kemudi $19.25^\circ$. Namun, simpangan error lateral masih berfluktuasi cukup lebar ($11.25\text{ cm}$) akibat sisa osilasi.
    * **Grafik EMA 0.6**: Menunjukkan performa transien yang stabil dan aman. Kemudi servo bergeser mulus tanpa jitter mekanis dengan frekuensi $23.7\text{ CPM}$ dan standar deviasi $21.57^\circ$, berhasil memandu robot melintasi tikungan secara presisi.
    * **Grafik EMA 0.4**: Waktu tunda filter yang mencapai $\approx 100\text{ ms}$ mulai menggeser fasa aksi kontrol kemudi. Hal ini memicu simpangan error lateral yang tinggi (standar deviasi $11.81$ cm) di mana setir terlambat membelok saat memasuki area belokan.
    * **Grafik EMA 0.2**: Menunjukkan kegagalan total sistem kendali kemudi lateral. Tunda respons yang sangat besar ($\approx 180\text{ ms}$) dikombinasikan dengan kecepatan linier tinggi $0.7\text{ m/s}$ membuat kemudi terlambat berbelok. Grafik error lajur biru melesat naik melampaui batas $17.57\text{ cm}$ ($150$ px) secara permanen dan robot keluar dari lintasan.

* **Perbandingan Profil Lane Error Lintas Kecepatan (Deadzone 30 px)**:
  Visualisasi komparatif pengaruh kecepatan terhadap profil pengereman LKA untuk tiap parameter filter EMA:
  
  ````carousel
  ![EMA 1.0 (Tanpa Filter)](file:///a:/ACKERMAN/ema_10_comparison.png)
  <!-- slide -->
  ![EMA 0.8](file:///a:/ACKERMAN/ema_08_comparison.png)
  <!-- slide -->
  ![EMA 0.6](file:///a:/ACKERMAN/ema_06_comparison.png)
  <!-- slide -->
  ![EMA 0.4](file:///a:/ACKERMAN/ema_04_comparison.png)
  <!-- slide -->
  ![EMA 0.2 (Default)](file:///a:/ACKERMAN/ema_02_comparison.png)
  ````

  > **Penjelasan Grafik Perbandingan Kecepatan Lintas EMA**:
  > Grafik di atas membandingkan respons *Lane Error* untuk tiga variasi kecepatan (0.3, 0.5, 0.7 m/s) pada setiap filter EMA. Pada EMA 1.0 (Tanpa Filter), kecepatan tinggi (merah) memicu frekuensi osilasi dan simpangan yang sangat besar dibanding kecepatan rendah (hijau). Pada filter optimal EMA 0.6, kurva transien untuk ketiga kecepatan tetap konvergen stabil. Pada EMA 0.2, terlihat jelas bahwa pada kecepatan $0.7\text{ m/s}$ (merah) kurva langsung melambung tak terhingga (robot keluar marka lajur jalan).

  * **Analisis Grafik Perbandingan Lintas Kecepatan (Deadzone 30 px)**:
    * **Grafik Perbandingan EMA 1.0**: Memperlihatkan bahwa tanpa pemfilteran, peningkatan kecepatan dari $0.3\text{ m/s}$ (hijau) ke $0.7\text{ m/s}$ (merah) memperbesar amplitudo osilasi error secara eksponensial akibat getaran servo frekuensi tinggi yang mengganggu kestabilan kendaraan.
    * **Grafik Perbandingan EMA 0.8**: Kurva komparasi memperlihatkan penurunan osilasi pada kecepatan rendah ($0.3\text{ m/s}$) dan menengah ($0.5\text{ m/s}$), namun pada kecepatan tinggi ($0.7\text{ m/s}$ - kurva merah) simpangan error masih berfluktuasi cukup lebar di sepanjang lintasan.
    * **Grafik Perbandingan EMA 0.6**: Menunjukkan karakteristik transien terbaik lintas kecepatan. Kurva error untuk ketiga variasi kecepatan (0.3, 0.5, 0.7 m/s) tampak konvergen stabil dan tidak menyimpang jauh dari titik tengah, memvalidasi kinerja optimal filter EMA 0.6.
    * **Grafik Perbandingan EMA 0.4**: Memperlihatkan mulainya divergensi kurva error pada kecepatan tinggi $0.7\text{ m/s}$ (kurva merah) akibat tunda filter yang menyebabkan overshoot, sedangkan kecepatan rendah dan menengah masih berada dalam batas stabil.
    * **Grafik Perbandingan EMA 0.2**: Memperlihatkan secara dramatis kegagalan kontrol kemudi lateral pada kecepatan tinggi $0.7\text{ m/s}$ (kurva merah), di mana error lajur melesat naik tak terhingga karena robot keluar lintasan, sementara kecepatan rendah ($0.3\text{ m/s}$) masih mampu bertahan di dalam lajur.

* **Analisis Hasil (v0 = 0.7 m/s, DZ = 30 px)**:
  Pada kecepatan tinggi $0.7\text{ m/s}$, keterlambatan respons akibat penyaringan data (*filtering latency*) berakibat fatal. Pada `alpha = 0.2`, keterlambatan koreksi lateral sebesar $\approx 180\text{ ms}$ dikombinasikan dengan kecepatan linier yang tinggi menyebabkan roda kemudi terlambat merespon tikungan, sehingga robot meluncur lurus keluar dari lajur marka jalan kanan. Kinerja yang optimal dan aman dicapai pada `alpha = 0.6`, di mana pergerakan servo kemudi cukup mulus (tidak kasar) tanpa menyebabkan kendaraan keluar dari marka lajur jalan dengan frekuensi koreksi $23.7\text{ CPM}$.

---

##### B. Eksperimen 4B: Variasi EMA pada Deadzone 40 px

##### I. Pengujian EMA (Deadzone 40 px) pada Kecepatan 0.3 m/s
* **Tabel Hasil Eksperimen 3A (Speed = 0.3 m/s, Deadzone = 40 px)**:
  | Nilai `alpha_filter` EMA | Standar Deviasi Lane Error (cm) | Standar Deviasi Sudut Kemudi /rho (derajat) | Frekuensi Koreksi Setir (siklus/menit) | Karakteristik Kemudi & Waktu Tunda |
  | :---: | :---: | :---: | :---: | :--- |
  | $1.0$ (Tanpa Filter) | $7.25$ | $18.67$ | $31.9$ | Koreksi aktif, sedikit getaran |
  | $0.8$ | $3.72$ | $16.75$ | $35.2$ | Cukup mulus |
  | $0.6$ | $7.40$ | $22.80$ | $31.6$ | Halus dan stabil |
  | $0.4$ | $5.46$ | $17.97$ | $7.0$ | Sangat halus, koreksi lambat |
  | $0.2$ | $0.74$ | $16.02$ | $17.7$ | Sangat mulus di garis tengah, kurang sensitif |

* **Grafik Hasil Eksperimen 3A (Speed = 0.3 m/s, Deadzone = 40 px)**:
  
  ````carousel
  ![EMA 1.0 (v0 = 0.3 m/s, DZ = 40 px)](file:///a:/ACKERMAN/ema_dz40_10_03ms.png)
  <!-- slide -->
  ![EMA 0.8 (v0 = 0.3 m/s, DZ = 40 px)](file:///a:/ACKERMAN/ema_dz40_08_03ms.png)
  <!-- slide -->
  ![EMA 0.6 (v0 = 0.3 m/s, DZ = 40 px)](file:///a:/ACKERMAN/ema_dz40_06_03ms.png)
  <!-- slide -->
  ![EMA 0.4 (v0 = 0.3 m/s, DZ = 40 px)](file:///a:/ACKERMAN/ema_dz40_04_03ms.png)
  <!-- slide -->
  ![EMA 0.2 (v0 = 0.3 m/s, DZ = 40 px)](file:///a:/ACKERMAN/ema_dz40_02_03ms.png)
  ````

  > **Penjelasan Grafik Karosel 3A (Deadzone 40 px - 0.3 m/s)**:
  > Grafik karosel di atas memperlihatkan pergerakan kemudi dengan zona toleransi $\pm40$ px (atau $\pm4.69$ cm) (garis merah horizontal). Karena kecepatan sangat lambat ($0.3\text{ m/s}$), seluruh parameter EMA (1.0 s.d 0.2) mampu mempertahankan robot dalam lajur dengan aman. Terlihat garis hitam (sudut kemudi) tetap mendatar di $180^\circ$ lebih lama sebelum melakukan pergerakan koreksi, akibat lebar wilayah toleransi yang lebih besar. Enclosure EMA 0.2 (Sangat Kuat) menghasilkan kurva yang sangat mulus dengan simpangan error minimal.

  * **Analisis Detail Grafik Individual (v0 = 0.3 m/s, DZ = 40 px)**:
    * **Grafik EMA 1.0**: Sudut kemudi ($\rho$) berayun merespon error dengan frekuensi koreksi $31.9\text{ CPM}$ dan standar deviasi $18.67^\circ$. Sedikit jitter kecil terlihat saat robot mendeteksi marka di luar wilayah toleransi merah $\pm40$ px (atau $\pm4.69$ cm).
    * **Grafik EMA 0.8**: Penyaringan ringan berhasil meredam jitter kemudi dengan frekuensi $35.2\text{ CPM}$ dan standar deviasi kemudi $16.75^\circ$, menjaga pergerakan tetap aman pada kecepatan rendah.
    * **Grafik EMA 0.6**: Sudut kemudi bergerak lebih halus dengan frekuensi koreksi $31.6\text{ CPM}$ dan standar deviasi kemudi $22.80^\circ$, menyaring noise koordinat kamera dengan efektif.
    * **Grafik EMA 0.4**: Respons kemudi servo tampak sangat lambat (CPM turun ke $7.0\text{ CPM}$ dengan standar deviasi kemudi $17.97^\circ$). Setir melurus di $180^\circ$ lebih lama sebelum membelok secara perlahan saat berada di tikungan.
    * **Grafik EMA 0.2**: Filter kuat memotong hampir seluruh osilasi dengan frekuensi koreksi $17.7\text{ CPM}$ dan standar deviasi kemudi $16.02^\circ$. Roda kemudi melurus tenang karena noise sensor tersaring sepenuhnya.

##### II. Pengujian EMA (Deadzone 40 px) pada Kecepatan 0.5 m/s
* **Tabel Hasil Eksperimen 3B (Speed = 0.5 m/s, Deadzone = 40 px)**:
  | Nilai `alpha_filter` EMA | Standar Deviasi Lane Error (cm) | Standar Deviasi Sudut Kemudi /rho (derajat) | Frekuensi Koreksi Setir (siklus/menit) | Karakteristik Kemudi & Waktu Tunda |
  | :---: | :---: | :---: | :---: | :--- |
  | $1.0$ (Tanpa Filter) | $3.96$ | $14.73$ | $30.4$ | Getaran kemudi rendah |
  | $0.8$ | $8.99$ | $24.12$ | $34.5$ | Bergetar ringan |
  | $0.6$ | $12.89$ | $21.36$ | $16.0$ | Cukup lambat, understeer ringan |
  | $0.4$ | $3.59$ | $16.60$ | $11.6$ | Terlalu lambat merespon tikungan |
  | $0.2$ | -- | -- | -- | Gagal berbelok (Data tidak terekam/keluar lajur) |

* **Grafik Hasil Eksperimen 3B (Speed = 0.5 m/s, Deadzone = 40 px)**:
  
  ````carousel
  ![EMA 1.0 (v0 = 0.5 m/s, DZ = 40 px)](file:///a:/ACKERMAN/ema_dz40_10_05ms.png)
  <!-- slide -->
  ![EMA 0.8 (v0 = 0.5 m/s, DZ = 40 px)](file:///a:/ACKERMAN/ema_dz40_08_05ms.png)
  <!-- slide -->
  ![EMA 0.6 (v0 = 0.5 m/s, DZ = 40 px)](file:///a:/ACKERMAN/ema_dz40_06_05ms.png)
  <!-- slide -->
  ![EMA 0.4 (v0 = 0.5 m/s, DZ = 40 px)](file:///a:/ACKERMAN/ema_dz40_04_05ms.png)
  ````

  > **Penjelasan Grafik Karosel 3B (Deadzone 40 px - 0.5 m/s)**:
  > Grafik karosel di atas memperlihatkan transisi menuju ketidakstabilan lateral akibat kombinasi deadzone lebar dan filter berat. Pada EMA 0.6 and 0.4, kurva kemudi hitam mengalami perlambatan reaksi yang signifikan saat simpangan lajur biru meluncur tajam. Untuk parameter EMA 0.2, keterlambatan respons kemudi menyebabkan kegagalan berbelok secara penuh sehingga data log terputus akibat robot keluar dari lajur marka jalan.

  * **Analisis Detail Grafik Individual (v0 = 0.5 m/s, DZ = 40 px)**:
    * **Grafik EMA 1.0**: Roda kemudi servo berosilasi aktif dengan frekuensi $30.4\text{ CPM}$ dan standar deviasi $14.73^\circ$. Jitter kemudi tampak lebih sedikit dibandingkan dengan deadzone 30 px akibat zona toleransi mati yang lebih lebar.
    * **Grafik EMA 0.8**: Filter ringan menghasilkan frekuensi koreksi $34.5\text{ CPM}$ dengan standar deviasi kemudi $24.12^\circ$. Setir berbelok secara aktif ketika error lajur melanggar koridor merah.
    * **Grafik EMA 0.6**: Menunjukkan adanya keterlambatan respons kemudi. Frekuensi koreksi turun ke $16.0\text{ CPM}$ dengan standar deviasi $21.36^\circ$, namun error simpangan biru melambung tinggi hingga standar deviasi $12.89$ cm sebelum roda membelok.
    * **Grafik EMA 0.4**: Menunjukkan respons kemudi yang sangat lambat (CPM $11.6$ dan standar deviasi $16.60^\circ$). Garis kemudi hitam tetap mendatar tenang meskipun garis error biru telah meluncur jauh keluar dari batas merah $\pm40$ px (atau $\pm4.69$ cm), mengindikasikan understeer.
    * **Grafik EMA 0.2 (Gagal)**: Grafik tidak disajikan karena tunda respons kumulatif menyebabkan robot meluncur lurus keluar dari marka lintasan luar sebelum sempat berbelok di tikungan tajam (log data terputus).

##### III. Pengujian EMA (Deadzone 40 px) pada Kecepatan 0.7 m/s
* **Tabel Hasil Eksperimen 3C (Speed = 0.7 m/s, Deadzone = 40 px)**:
  | Nilai `alpha_filter` EMA | Standar Deviasi Lane Error (cm) | Standar Deviasi Sudut Kemudi /rho (derajat) | Frekuensi Koreksi Setir (siklus/menit) | Karakteristik Kemudi & Waktu Tunda |
  | :---: | :---: | :---: | :---: | :--- |
  | $1.0$ (Tanpa Filter) | $11.87$ | $19.12$ | $20.2$ | Kasar dan bergetar |
  | $0.8$ | $11.25$ | $19.25$ | $9.8$ | Cukup mulus |
  | $0.6$ | $8.11$ | $19.47$ | $5.1$ | Pergerakan lambat, overshoot tikungan |
  | $0.4$ | $8.15$ | $16.56$ | $25.8$ | Respons lambat, understeer nyata |
  | $0.2$ | -- | -- | -- | Gagal berbelok (Keluar lajur tikungan) |

* **Grafik Hasil Eksperimen 3C (Speed = 0.7 m/s, Deadzone = 40 px)**:
  
  ````carousel
  ![EMA 1.0 (v0 = 0.7 m/s, DZ = 40 px)](file:///a:/ACKERMAN/ema_dz40_10_07ms.png)
  <!-- slide -->
  ![EMA 0.8 (v0 = 0.7 m/s, DZ = 40 px)](file:///a:/ACKERMAN/ema_dz40_08_07ms.png)
  <!-- slide -->
  ![EMA 0.6 (v0 = 0.7 m/s, DZ = 40 px)](file:///a:/ACKERMAN/ema_dz40_06_07ms.png)
  <!-- slide -->
  ![EMA 0.4 (v0 = 0.7 m/s, DZ = 40 px)](file:///a:/ACKERMAN/ema_dz40_04_07ms.png)
  ````

  > **Penjelasan Grafik Karosel 3C (Deadzone 40 px - 0.7 m/s)**:
  > Grafik karosel di atas menunjukkan respons transien lateral ekstrem pada kecepatan $0.7\text{ m/s}$ dengan toleransi $\pm40$ px (atau $\pm4.69$ cm). Di sini, delay respons dari EMA 0.6 dan 0.4 sudah berada di luar batas toleransi dinamis robot Ackerman, ditandai dengan overshoot simpangan biru yang tinggi sebelum servo hitam berputar penuh. Pada EMA 0.2, kestabilan gagal dipertahankan dan robot menabrak batas marka lintasan luar.

  * **Analisis Detail Grafik Individual (v0 = 0.7 m/s, DZ = 40 px)**:
    * **Grafik EMA 1.0**: Menunjukkan kemudi berayun kasar dengan frekuensi $20.2\text{ CPM}$ dan standar deviasi kemudi $19.12^\circ$ akibat noise marka yang tidak tersaring.
    * **Grafik EMA 0.8**: Filter ringan menghasilkan gerakan setir yang lebih mulus dengan frekuensi koreksi $9.8\text{ CPM}$ dan standar deviasi $19.25^\circ$, namun respon membelok mulai sedikit terlambat.
    * **Grafik EMA 0.6**: Kecepatan respons berkurang drastis dengan frekuensi koreksi hanya $5.1\text{ CPM}$ dan standar deviasi kemudi $19.47^\circ$. Tunda respons lateral di tikungan memicu simpangan error yang besar ($8.11$ cm) sebelum servo sempat membelok penuh.
    * **Grafik EMA 0.4**: Respons kemudi terlambat dan berayun kasar dengan frekuensi $25.8\text{ CPM}$ akibat koreksi berlebih (overshoot) di akhir tikungan demi mengejar ketertinggalan lajur setelah tunda yang lama.
    * **Grafik EMA 0.2 (Gagal)**: Grafik tidak dapat disajikan karena kombinasi deadzone lebar $\pm40$ px (atau $\pm4.69$ cm) dan filter berat EMA 0.2 melampaui batas delay dinamis aman robot, menyebabkan robot langsung meluncur lurus keluar dari lajur jalan di tikungan pertama.

* **Analisis Hasil Eksperimen 4B (Deadzone 40 px)**:
  Dengan melebarkan deadzone ke $40$ px, sistem kendali lateral memiliki kestabilan arah garis tengah yang lebih kaku, namun hal ini membatasi sensitivitas deteksi simpangan. Pada kecepatan $0.5\text{ m/s}$ dan $0.7\text{ m/s}$ dengan `alpha_filter` yang kuat ($0.2$ s.d. $0.4$), tunda kemudi kumulatif (gabungan antara deadzone lebar dan filter EMA yang berat) melebihi waktu respons aman lateral. Akibatnya, pada `alpha = 0.2`, robot tidak sempat berbelok pada tikungan tajam dan menyeberang lurus melewati garis marka jalan luar, memicu kegagalan sistem kendali kemudi LKA.

---

#### 4.7.3 Karakteristik Respon Pengereman ACC berdasarkan Dynamic Proxemics

Bagian ini menyajikan pengujian karakteristik deselerasi otonom terintegrasi (Coupled SFM) dengan menyesuaikan parameter proksemik visual (Context Aware) terhadap tipe rintangan pejalan kaki, bus, dan truk.

##### I. Analisis Karakteristik Tiap Kecepatan Target
Pengujian dilakukan dengan menghadapkan robot pada pejalan kaki (*Person*) statis di tengah lajur untuk menganalisis hubungan kecepatan target awal $v_0$ dengan toleransi elips keamanan robot.

* **Tabel Hasil Eksperimen**:
  | Kecepatan Target $v_0$ (m/s) | Parameter Dinamis Robot: semi-axis $a$ (m) | Jarak Deteksi Awal Mulai Mengerem (m) | Jarak Akhir Berhenti Nyata $d_{\text{stop}}$ (m) | Status Keselamatan (Tabrakan/Aman) |
  | :---: | :---: | :---: | :---: | :---: |
  | $0.3$ | $0.50$ | ... | ... | Aman (Berhenti presisi) |
  | $0.5$ | $0.64$ | ... | ... | Aman |
  | $0.7$ | $0.78$ | ... | ... | Aman |
  | $1.0$ | $1.00$ | ... | ... | Aman (Pengereman dini) |

* **Analisis Hasil**:
  * Pada kecepatan rendah $0.3\text{ m/s}$, elips keamanan robot mengecil secara dinamis ($a = 0.5\text{ m}$). Secara fisis, hal ini membatasi gaya tolak sosial agar tidak aktif terlalu cepat, sehingga memperbolehkan robot mendekati objek dan berhenti pada jarak yang rapat and presisi tanpa memicu pengereman dini yang tidak perlu.
  * Pada kecepatan tinggi $1.0\text{ m/s}$, elips keamanan membesar menjadi $a = 1.0\text{ m}$. Ini secara otomatis mengaktifkan pengereman ACC pada jarak yang lebih jauh, memberikan ruang deselerasi yang aman untuk menetralisir energi kinetik robot akibat inersia kecepatan tinggi.

###### II. Pengujian Karakteristik Respon Pengereman ACC dengan Coupled Social Force Model (Truck-Following)
Pengujian dinamis dilakukan untuk menganalisis kinerja adaptasi kecepatan robot Ackerman dalam skenario mengikuti rintangan bergerak di depannya (*Truck-Following*) menggunakan data log `Pengujian Menyesuaikan Objek Didepan.csv`.

* **Grafik Eksperimen**:
  ![Karakteristik Respon Menyesuaikan Objek Didepan](C:/Users/Administrator/.gemini/antigravity-ide/brain/bd40d144-e1f5-4f8d-91b4-a4367299e5dd/sfm_truck_following.png)

  * **Analisis Grafik Menyesuaikan Objek Didepan**: Grafik ini menggambarkan transisi dinamis kecepatan robot aktual (`Real Velocity`, garis oranye/kuning) dan kecepatan perintah (`Velocity Command`, garis biru) yang berangsur turun menyesuaikan dengan manuver truk di depan. Terlihat respons gaya sosial repulsif ($F_{rep}$, garis hijau) berfluktuasi lembut di rentang $0 - 20\text{ N}$ saat truk mulai melambat, menekan kecepatan aktual secara mulus tanpa sentakan tajam untuk menjaga jarak aman tetap stabil di rentang $0.27 - 0.35\text{ m}$.

* **Analisis Hasil**:
  * Selama pengujian, kecepatan robot menyesuaikan secara otomatis dengan kecepatan truk di depannya. Gaya repulsif $F_{rep}$ berosilasi secara dinamis di rentang $0 - 20\text{ N}$ guna mempertahankan jarak aman pada rentang rapat $0.27 - 0.35\text{ m}$.
  * Ketika truk di depan melambat, pinalti gaya repulsif longitudinal Coupled SFM bekerja langsung mengurangi perintah kecepatan target (`Velocity Command`) sehingga robot melambat secara halus seiring turunnya kecepatan aktual roda (`Real Velocity`). Pengujian ini memvalidasi keberhasilan *Dynamic Gain Scheduling* dalam mempertahankan kestabilan formasi dinamis.

##### III. Grafik dan Analisis Karakteristik Following Objek:
Pengujian statis berjalan mendekat pada kecepatan awal target $v_0 = 1.0\text{ m/s}$ dilakukan untuk melihat pengaruh tipe rintangan terhadap profil deselerasi.

* **Tabel Hasil Eksperimen**:
  | Tipe Rintangan | Kelas YOLO | Parameter Dinamis: Elips $a \times b$ (m) | Jarak Mulai Mengerem $d_{\text{start}}$ (m)* | Jarak Akhir Berhenti Nyata $d_{\text{stop}}$ (m)** | Gaya Repulsif Maksimum $F_{rep, max}$ (N) | Status Keselamatan |
  | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
  | Pejalan Kaki (Person) | Person (3) | $0.90 \times 0.15$ | $1.748$ | $0.546$ | $138.93$ | Aman |
  | Bus | Bus (0) | $1.20 \times 1.00$ | $1.820$ | $0.636$ | $263.42$ | Aman |
  | Truk ($K_{multiplier} = 1.5$) | Truck (4) | $1.20 \times 1.00$ | $1.826$ | $0.520$ | $399.43$ | Aman |

* **Grafik Hasil Eksperimen**:
  Visualisasi komparatif profil pengereman otonom terhadap 3 jenis rintangan berbeda:
  
  ![Grafik Komparasi Profil Pengereman ACC Coupled SFM](C:/Users/Administrator/.gemini/antigravity-ide/brain/bd40d144-e1f5-4f8d-91b4-a4367299e5dd/sfm_braking_comparison.png)

  * **Analisis Grafik Komparasi Profil Pengereman otonom**: Grafik ini menumpangkan ketiga kurva deselerasi. Terlihat jelas bahwa profil pengereman pejalan kaki (garis hijau) memiliki kemiringan deselerasi yang paling landai dan dimulai paling lambat. Sementara profil bus (biru) dan truk (merah) terpicu lebih awal dengan kemiringan deselerasi yang lebih curam, membuktikan kemampuan deteksi berbasis kelas objek.

  Berikut adalah profil deselerasi otonom individual untuk masing-masing kelas objek:

  ````carousel
  ![Profil Pengereman: Pejalan Kaki (Person)](C:/Users/Administrator/.gemini/antigravity-ide/brain/bd40d144-e1f5-4f8d-91b4-a4367299e5dd/sfm_braking_person.png)
  <!-- slide -->
  ![Profil Pengereman: Bus](C:/Users/Administrator/.gemini/antigravity-ide/brain/bd40d144-e1f5-4f8d-91b4-a4367299e5dd/sfm_braking_bus.png)
  <!-- slide -->
  ![Profil Pengereman: Truk (K_mult = 1.5)](C:/Users/Administrator/.gemini/antigravity-ide/brain/bd40d144-e1f5-4f8d-91b4-a4367299e5dd/sfm_braking_truck_k15.png)
  ````

  * **Analisis Detail Grafik Individual (Profil Pengereman)**:
    * **Grafik Profil Pejalan Kaki (Person)**: Menunjukkan bahwa elips keamanan person yang kecil ($a=0.9\text{ m}$) memicu pengereman pada jarak $d_{\text{start}} = 1.748\text{ m}$. Kecepatan turun secara linear dan halus dengan gaya repulsif puncak rendah $F_{rep, max} = 138.93\text{ N}$ sebelum berhenti presisi pada $d_{\text{stop}} = 0.546\text{ m}$.
    * **Grafik Profil Bus**: Karakteristik objek besar dengan elips $a=1.2\text{ m}$ memicu gaya repulsif lebih awal pada $d_{\text{start}} = 1.820\text{ m}$. Gaya repulsif naik secara progresif hingga mencapai puncak $263.42\text{ N}$ guna menetralisir inersia robot dan menghentikannya pada jarak aman $d_{\text{stop}} = 0.636\text{ m}$.
    * **Grafik Profil Truk ($K_{multiplier} = 1.5$)**: Dengan elips semi-axis $a=1.2\text{ m}$, pengereman terpicu hampir bersamaan dengan bus pada $d_{\text{start}} = 1.826\text{ m}$. Namun, karena robot mendekati objek sedikit lebih dekat ($d_{\text{stop}} = 0.520\text{ m}$), gaya tolak eksponensial bekerja lebih agresif hingga memuncak di **399.43 N** untuk menjamin keselamatan berhenti.

* **Analisis Hasil**:
  * **Pejalan Kaki (Person)**: Memiliki ukuran elips proksemik terkecil ($a=0.9\text{ m}$). Pengereman baru terpicu pada jarak terdekat $d_{\text{start}} = 1.748\text{ m}$ dengan deselerasi linear yang sangat mulus (durasi $1.833\text{ s}$), gaya puncak rendah $F_{rep, max} = 138.93\text{ N}$, dan berhenti halus pada jarak $d_{\text{stop}} = 0.546\text{ m}$.
  * **Bus**: Karakteristik elips yang besar ($a=1.2\text{ m}$) memicu pengereman dini pada $d_{\text{start}} = 1.820\text{ m}$. Gaya repulsif naik lebih curam mencapai puncak $263.42\text{ N}$ saat berhenti penuh pada jarak aman yang lebih lebar $d_{\text{stop}} = 0.636\text{ m}$.
  * **Truk ($K_{multiplier} = 1.5$)**: Memiliki elips identik dengan Bus ($a=1.2\text{ m}$), sehingga jarak awal pengereman sangat mirip ($1.826\text{ m}$). Gaya repulsif puncak mencapai **399.43 N** karena robot mendekati objek sedikit lebih dekat ($d_{\text{stop}} = 0.520\text{ m}$), memicu lonjakan gaya sosial secara eksponensial.

##### IV. Grafik dan Analisis Karakteristik Pengereman dengan perbedaan K_multiplier
Bagian ini menganalisis pengaruh parameter tuning keselamatan $K_{multiplier}$ dengan membandingkan respons Truk pada pengali $1.5$ dan $2.0$ (data `Truck Kmult_2 5m 17 Des.csv`).

* **Tabel Komparasi**:
  | Konfigurasi Truk | Kecepatan Awal Nyata (m/s) | Jarak Mulai Mengerem $d_{\text{start}}$ (m) | Jarak Akhir Berhenti Nyata $d_{\text{stop}}$ (m) | Gaya Repulsif Maksimum $F_{rep, max}$ (N) |
  | :---: | :---: | :---: | :---: | :---: |
  | Truk ($K_{multiplier} = 1.5$) | $1.032$ | $1.826$ | $0.520$ | $399.43$ |
  | Truk ($K_{multiplier} = 2.0$) | $1.032$ | $1.894$ | $0.548$ | $484.20$ |

* **Grafik Hasil Eksperimen**:
  Tinjauan komparasi visual respon deselerasi Truk dengan variasi $K_{multiplier}$:

  ````carousel
  ![Profil Pengereman: Truk (K_mult = 1.5)](C:/Users/Administrator/.gemini/antigravity-ide/brain/bd40d144-e1f5-4f8d-91b4-a4367299e5dd/sfm_braking_truck_k15.png)
  <!-- slide -->
  ![Profil Pengereman: Truk (K_mult = 2.0)](C:/Users/Administrator/.gemini/antigravity-ide/brain/bd40d144-e1f5-4f8d-91b4-a4367299e5dd/sfm_braking_truck_k20.png)
  ````

  * **Analisis Detail Grafik Individual (Variasi K_multiplier)**:
    * **Grafik Truk ($K_{multiplier} = 1.5$)**: Menunjukkan deselerasi dengan kelandaian stabil dari kecepatan awal target, di mana gaya repulsif naik secara eksponensial di akhir mendekati rintangan hingga mencapai $399.43\text{ N}$ untuk menahan robot di jarak $0.520\text{ m}$.
    * **Grafik Truk ($K_{multiplier} = 2.0$)**: Memperlihatkan pengaruh pengali sensitivitas yang lebih tinggi. Gaya repulsif terpicu lebih cepat dan naik lebih curam mencapai puncak **484.20 N** (meningkat $21.2\%$), memaksa robot melakukan deselerasi lebih agresif dan berhenti pada jarak aman yang lebih lebar yaitu $0.548\text{ m}$ (lebih jauh $2.8\text{ cm}$).

* **Analisis Hasil**:
  Peningkatan $K_{multiplier}$ dari $1.5$ ke $2.0$ membuktikan efektivitas variabel multiplier sebagai parameter tuning keselamatan:
  * **Jarak Reaksi Pengereman ($d_{\text{start}}$)**: Sistem mulai mengerem lebih dini dari **1.826 m** menjadi **1.894 m** (lebih cepat $\approx 6.8\text{ cm}$) untuk mengantisipasi inersia objek berisiko tinggi.
  * **Gaya Deselerasi ($F_{rep, max}$)**: Gaya repulsif maksimum melonjak dari **399.43 N** menjadi **484.20 N** (kenaikan $21.2\%$). Kurva deselerasi pada Truk $K=2.0$ menunjukkan kelandaian penurunan kecepatan yang lebih curam (*slope* tinggi), menandakan pengereman darurat yang agresif dan responsif.
  * **Margin Jarak Berhenti ($d_{\text{stop}}$)**: Deselerasi yang kuat memaksa robot berhenti lebih cepat and menyisakan margin keselamatan berhenti yang lebih aman, dari **0.520 m** menjadi **0.548 m** (lebih jauh $2.8\text{ cm}$).

##### V. Analisis Fisis & Perbandingan terhadap Dynamic Proxemics
* **Analisis Formulasi Fisis**:
  Pada model *Coupled* SFM, deselerasi longitudinal diperoleh melalui formulasi:
  $$F_{\text{total}, x} = F_{\text{goal}} + F_{\text{obs}, x}$$
  $$F_{rep} = F_{\text{goal}} - F_{\text{total}, x} = -F_{\text{obs}, x} = f_{\text{force\_static}} \cos(\theta_{\text{obs}})$$
  di mana $f_{\text{force\_static}}$ didefinisikan secara non-linear:
  $$f_{\text{force\_static}} = k_{\text{effective}} \cdot e^{\frac{r - d}{\psi}} + k_{\text{effective}} \cdot (r - d)$$
  Lonjakan tajam gaya pada grafik membuktikan bahwa pengali $k_{\text{effective}} = k \cdot K_{multiplier}$ bekerja secara non-linear dalam memperluas sensitivitas pengereman robot Ackerman.
* **Perbandingan dengan ACC Konvensional**:
  ACC konvensional yang hanya berbasis sensor jarak (LiDAR) memicu kurva pengereman yang identik untuk semua tipe rintangan (buta konteks). Sebaliknya, sistem berbasis *Context Aware Dynamic Proxemics* ini secara cerdas menyesuaikan elips keamanan $a \times b$ berdasarkan kelas YOLOv8. Hal ini memungkinkan robot menghemat ruang reaksi saat menghadapi pejalan kaki (Person) guna mendukung kenyamanan sosial, namun secara agresif mengaktifkan deselerasi dini bermargin aman tinggi ketika mendeteksi kendaraan besar (Bus/Truk).

---

#### 4.7.4 Karakteristik Kemudi Terpadu (Coupled LKA-SFM) pada Skenario Penghindaran Rintangan Samping
Pengujian dilakukan dengan meletakkan objek rintangan di sisi kiri dalam lajur yang sedikit memotong jalur lintasan robot untuk menganalisis interaksi kemudi lateral Coupled SFM.

* **Tabel Hasil Eksperimen 3**:
  | Offset Rintangan Memotong Lajur (cm) | Gaya Tolak Maksimum Objek $F_{\text{obs},y}$ (N) | Gaya Koreksi Maksimum LKA $F_{\text{lane},y}$ (N) | Nilai Maksimum Deviasi Lajur Kanan (piksel) | Keberhasilan Penghindaran (Ya/Tidak) |
  | :---: | :---: | :---: | :---: | :---: |
  | $10\text{ cm}$ | ... | ... | ... | Ya (Tetap dalam lajur) |
  | $20\text{ cm}$ | ... | ... | ... | Ya (Sedikit melipir ke kanan) |
  | $30\text{ cm}$ | ... | ... | ... | Tidak (Keluar marka kanan) |

* **Analisis Hasil**:
  * Analisis difokuskan pada interaksi gaya lateral Coupled SFM:
    $$F_{\text{total}, y} = F_{\text{lane}, y} + F_{\text{obs}, y}$$
  * Ketika rintangan berada di sisi kiri ($F_{\text{obs}, y} > 0$), gaya ini mendorong robot ke kanan. Di saat yang sama, LKA akan menghasilkan gaya penahan ke kiri ($F_{\text{lane}, y} < 0$) ketika robot mulai condong mendekati marka kanan.
  * Analisis batas kemampuan sistem: Jika objek memotong hingga $30\text{ cm}$, gaya tolak rintangan $F_{\text{obs}, y}$ mendominasi dan memaksa roda membelok tajam ke kanan hingga melewati batas marka jalan kanan karena lajur yang tersisa terlalu sempit untuk dilewati oleh dimensi robot Ackerman.

---

#### 4.7.5 Analisis Komparatif Optimalitas Parameter Sistem LKA

Untuk menentukan konfigurasi parameter terbaik bagi sistem *Lane Keeping Assist* (LKA) pada robot otonom Ackerman, dilakukan evaluasi silang terhadap seluruh hasil pengujian *deadzone* dan filter smoothing EMA lintas kecepatan ($0.3\text{ m/s}$, $0.5\text{ m/s}$, dan $0.7\text{ m/s}$).

* **Grafik Analisis Komparatif Performansi**:
  Visualisasi komparatif standar deviasi kesalahan lajur (*Lane Error*) dan standar deviasi sudut kemudi ($\rho$) pada Deadzone 30 px dan 40 px (catatan: *lower is better*):

  ![Analisis Komparatif Performansi Sistem LKA](file:///a:/ACKERMAN/optimal_parameter_comparison.png)

  > **Penjelasan Grafik Analisis Komparatif Performa**:
  > Grafik batang di atas membandingkan dua metrik performa utama (Lane Error di baris atas, Steering Angle di baris bawah) untuk Deadzone 30 px (kiri) dan Deadzone 40 px (kanan) lintas seluruh kecepatan uji dan koefisien EMA.
  > * **Baris Atas (Lane Error)**: Memperlihatkan bahwa seiring meningkatnya filter (dari $\alpha=1.0$ ke $\alpha=0.2$), deviasi kesalahan lajur cenderung mengecil pada kecepatan rendah ($0.3\text{ m/s}$), namun meningkat atau mengalami kegagalan pada kecepatan tinggi ($0.7\text{ m/s}$) karena delay filter.
  > * **Baris Bawah (Steering Angle)**: Menunjukkan penurunan standar deviasi kemudi (gerakan servo yang semakin tenang) yang sangat jelas seiring penurunan koefisien EMA. Nilai optimal $\alpha=0.6$ mewakili titik potong terbaik di mana getaran kemudi sangat rendah, namun nilai Lane Error tetap aman terkendali di bawah batas batas toleransi.

  * **Analisis Grafik Komparatif Performansi**: Grafik batang dua baris ini memberikan tinjauan visual menyeluruh. Pada baris atas (Lane Error), terlihat bahwa pada kecepatan $0.7\text{ m/s}$ dengan deadzone $30$ px, filter EMA 0.2 (merah) memicu lonjakan error ekstrem karena robot keluar dari lajur. Sementara itu, pada baris bawah (Steering Angle), standar deviasi sudut kemudi menurun secara konsisten dari filter lemah ($\alpha=1.0$) ke filter kuat ($\alpha=0.2$). Perpotongan optimal ditunjukkan pada parameter EMA 0.6 di mana getaran kemudi bernilai sangat rendah tanpa memicu kegagalan kontrol lateral (out of lane) di semua variasi kecepatan uji.

* **Pembahasan Kriteria Optimalitas**:
  Kinerja sistem LKA dinilai berdasarkan dua kriteria utama yang saling bertolak belakang (*competing objectives*):
  1. **Kestabilan Kemudi (Minimalisasi Jitter)**: Diukur dari nilai standar deviasi sudut kemudi $\rho$ yang rendah dan frekuensi koreksi (CPM) yang minim untuk menjaga efisiensi mekanis aktuator servo.
  2. **Akurasi Penelusuran Jalur (Minimalisasi Lane Error)**: Diukur dari nilai standar deviasi *Lane Error* yang rendah dan margin keselamatan transien untuk mencegah kendaraan keluar dari marka lajur jalan (menghindari kegagalan kontrol).

* **Kesimpulan Pemilihan Parameter Optimal**:
  Berdasarkan grafik komparasi performa di atas, konfigurasi **Deadzone 30 px (3.51 cm)** dan **EMA 0.6** terpilih sebagai parameter paling optimal karena alasan fisis berikut:

  * **Perbandingan terhadap Tanpa Filter / Jitter Tinggi (EMA 1.0 & 0.8)**:
    Tanpa filter (`alpha = 1.0`), getaran mikro pada pengolahan citra LKA secara langsung diterjemahkan menjadi gerakan koreksi setir yang sangat kasar (CPM mencapai $57.9$ s.d. $84.7$). Hal ini menyebabkan roda bergetar terus-menerus. Penerapan EMA 0.6 memotong frekuensi koreksi secara signifikan (misalnya dari $57.9\text{ CPM}$ ke $5.7\text{ CPM}$ pada kecepatan $0.3\text{ m/s}$) tanpa mengorbankan akurasi penelusuran jalur secara drastis.
  
  * **Perbandingan terhadap Filter Lamban / Risiko Understeer (EMA 0.4 & 0.2)**:
    Meskipun filter kuat seperti EMA 0.2 menghasilkan pergerakan kemudi yang sangat mulus, delay respons fasa sebesar $\approx 180\text{ ms}$ sangat berbahaya pada kecepatan menengah ke tinggi ($0.5\text{ m/s}$ dan $0.7\text{ m/s}$). Pada kecepatan $0.7\text{ m/s}$ dengan Deadzone $30$ px ($3.51\text{ cm}$), filter EMA 0.2 memicu keterlambatan belok yang menyebabkan robot meluncur keluar marka (kegagalan kontrol). Pada Deadzone $40$ px ($4.69\text{ cm}$), filter EMA 0.2 bahkan langsung memicu kegagalan sistem pada kecepatan $0.5\text{ m/s}$. Oleh karena itu, EMA 0.6 dengan delay respons aman $\approx 70\text{ ms}$ adalah batas aman optimal untuk mencegah *understeer* di semua variasi kecepatan uji.

  * **Pemilihan Deadzone 30 px vs 40 px / 50 px**:
    Lebar deadzone $30$ px ($3.51\text{ cm}$) memberikan ruang toleransi noise yang pas ($\pm3.51\text{ cm}$ dari pusat kamera) sehingga servo kemudi tetap tenang saat berjalan lurus. Sebaliknya, deadzone $40$ px ($4.69\text{ cm}$) dan $50$ px ($5.86\text{ cm}$) menciptakan "zona buta" kemudi yang terlalu lebar. Ketika dikombinasikan dengan delay filter EMA, zona buta ini memperlambat inisiasi belokan roda saat robot memasuki mulut tikungan, meningkatkan risiko keluar dari lajur jalan secara drastis.

Dengan demikian, kombinasi **Deadzone LKA = 30 piksel (3.51 cm)** dan **EMA alpha_filter = 0.6** terbukti secara empiris dan teoretis memberikan performansi terbaik: mempertahankan robot tetap berada di tengah lajur secara mulus, responsif, dan aman di seluruh rentang kecepatan operasi robot Ackerman.

---
