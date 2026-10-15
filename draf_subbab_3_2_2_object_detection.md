# Draf Bab 3.2.2: Perancangan Object Detection dan Fusi Sensor LiDAR-Kamera

Berikut adalah draf tulisan ilmiah dalam format sub-bab buku/laporan proyek akhir:

---

### 3.2.2 Perancangan Object Detection dan Fusi Sensor LiDAR-Kamera

Perancangan sub-sistem deteksi objek pada penelitian ini tidak hanya berfokus pada pengenalan kelas rintangan secara visual, melainkan juga mengintegrasikannya dengan informasi spasial jarak dan sudut secara presisi. Kamera konvensional memiliki keunggulan dalam menyajikan detail tekstur, bentuk, dan warna objek dua dimensi (2D) pada bidang citra, namun memiliki keterbatasan intrinsik dalam mengestimasi jarak kedalaman objek secara akurat. Sebaliknya, sensor LiDAR (*Light Detection and Ranging*) mampu menghasilkan pengukuran jarak dan sudut rintangan tiga dimensi (3D) yang sangat akurat, tetapi miskin akan informasi semantik kelas objek. Untuk mengatasi keterbatasan masing-masing sensor tersebut, pada sub-sistem ini dirancang sebuah metode fusi sensor tingkat tinggi (*high-level sensor fusion*) yang menggabungkan kecerdasan visual dari algoritma deep learning YOLOv8-segmentasi dengan presisi spasial awan titik (*point cloud*) dari sensor RPLidar.

#### 1. Sinkronisasi Temporal Sensor (Temporal Synchronization)
Sebelum proses fusi spasial dilakukan, perbedaan laju bingkai (*frame rate*) dan waktu akuisisi data (*clock skew*) dari kamera dan LiDAR harus diselesaikan terlebih dahulu melalui proses sinkronisasi temporal. Kamera bekerja secara asinkronus dengan laju sampling sekitar 30 Hz, sementara LiDAR bekerja dengan frekuensi putaran motor sekitar 5 hingga 10 Hz. Di dalam sistem operasi robot (ROS), dirancang sebuah node khusus bernama `time_sync_node` yang mengimplementasikan pustaka `message_filters` dengan kebijakan waktu perkiraan (*ApproximateTime*). 

Secara algoritma, node ini mendengarkan topik sensor mentah `/camera` dan `/scan`, lalu menyimpannya ke dalam antrean memori sementara (*buffer*) berukuran $10$. Kebijakan *ApproximateTime* akan membandingkan timestamp dari header pesan kamera ($t_{\text{image}, k}$) dan LiDAR ($t_{\text{scan}, j}$) pada setiap siklus datangnya pesan untuk mencari pasangan sensor dengan selisih waktu mutlak terkecil:
$$\Delta t = |t_{\text{image}, k} - t_{\text{scan}, j}|$$

Jika $\Delta t$ berada di bawah batas toleransi maksimum ($\Delta t \le \epsilon$), maka pasangan data sensor tersebut dinyatakan sinkron dan diterbitkan kembali secara bersamaan melalui topik baru, yaitu `/camera_sync` dan `/scan_sync`. Node deteksi objek utama kemudian hanya akan mengonsumsi topik tersinkronisasi ini untuk memastikan bahwa fusi spasial dilakukan pada keadaan fisik lingkungan yang sama.

#### 2. Perancangan Proyeksi Spasial 3D ke 2D (Spasial Calibration)
Untuk memetakan koordinat spasial titik laser LiDAR ke bidang citra kamera, diperlukan proses kalibrasi spasial ekstrinsik dan intrinsik. Sensor LiDAR menghasilkan data koordinat polar dua dimensi $[r_i, \theta_i]$ pada bidang horizontal datar. Tahap pertama dari kalibrasi spasial adalah mengonversi koordinat polar tersebut menjadi koordinat kartesian 3D pada frame LiDAR ($P_L$) dengan mengasumsikan komponen ketinggian $z_L = 0.0$:
$$P_L = \begin{bmatrix} x_L \\ y_L \\ z_L \end{bmatrix} = \begin{bmatrix} r_i \cdot \cos(\theta_i) \\ r_i \cdot \sin(\theta_i) \\ 0.0 \end{bmatrix}$$

Di mana:
*   $P_L$ adalah vektor koordinat 3D titik LiDAR pada kerangka acuan lokal sensor (meter).
*   $r_i$ adalah nilai jarak aman yang terukur oleh pancaran laser ke-$i$ (meter).
*   $\theta_i$ adalah sudut pancar relatif laser ke-$i$ terhadap sumbu hadap sensor (radian).

Tahap kedua adalah mentransformasikan koordinat $P_L$ tersebut ke kerangka acuan kamera 3D ($P_C$) menggunakan matriks transformasi rigid ekstrinsik yang terdiri dari matriks rotasi ($R$) dan vektor translasi ($T$):
$$P_C = \begin{bmatrix} x_C \\ y_C \\ z_C \end{bmatrix} = R \cdot P_L + T$$

Di mana:
*   $P_C$ adalah vektor koordinat 3D titik laser pada kerangka acuan lokal kamera (meter).
*   $R$ adalah matriks rotasi ekstrinsik ukuran $3 \times 3$ yang menyelaraskan orientasi sudut yaw, pitch, dan roll antara sensor LiDAR dan kamera.
*   $T$ adalah vektor translasi ekstrinsik ukuran $3 \times 1$ yang merepresentasikan posisi offset offset fisik sensor LiDAR terhadap kamera (meter).

Matriks rotasi ($R$) dan translasi ($T$) yang diperoleh dari proses kalibrasi fisik sensor pada robot ini bernilai:
$$R = \begin{bmatrix} -0.04006809 & 0.99906152 & 0.01645081 \\ 0.02362902 & 0.01740683 & -0.99956924 \\ -0.99891752 & -0.03966211 & -0.02430430 \end{bmatrix}, \quad T = \begin{bmatrix} -0.01693666 \\ 0.03729649 \\ -0.06359766 \end{bmatrix}$$

Tahap ketiga adalah memproyeksikan titik 3D kamera ($P_C$) ke koordinat citra homogen ($P_I$) menggunakan matriks intrinsik kamera ($K$):
$$P_I = \begin{bmatrix} u' \\ v' \\ w' \end{bmatrix} = K \cdot P_C = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} x_C \\ y_C \\ z_C \end{bmatrix}$$

Di mana:
*   $P_I$ adalah vektor koordinat citra homogen.
*   $K$ adalah matriks intrinsik kamera yang merepresentasikan karakteristik lensa internal.
*   $f_x, f_y$ adalah panjang fokus lensa kamera pada sumbu piksel horizontal dan vertikal, bernilai $f_x = 939.988307\text{ px}$ dan $f_y = 945.520654\text{ px}$.
*   $c_x, c_y$ adalah titik pusat optik bidang sensor gambar kamera, bernilai $c_x = 337.746494\text{ px}$ dan $c_y = 221.072054\text{ px}$.

Tahap terakhir adalah membagi koordinat homogen dengan komponen skala kedalaman ($w' = z_C$) untuk mendapatkan koordinat piksel 2D aktual $(u, v)$ pada gambar:
$$u = \frac{u'}{w'} = \frac{f_x \cdot x_C}{z_C} + c_x$$
$$v = \frac{v'}{w'} = \frac{f_y \cdot y_C}{z_C} + c_y$$

Titik piksel $(u, v)$ ini dinyatakan valid jika berada di dalam batas resolusi sensor gambar kamera (lebar $640\text{ px}$ dan tinggi $480\text{ px}$), serta memiliki kedalaman positif ($z_C > 0$):
$$0 \le u < 640 \quad \land \quad 0 \le v < 480 \quad \land \quad z_C > 0$$

#### 3. Logika Perhitungan Estimasi Jarak Fusi Tiga-Tingkat (Adaptive Distance Estimation)
Setelah proses pelacakan objek diselesaikan oleh model deep learning YOLOv8-seg, sistem akan menghasilkan kotak pembatas (*bounding box*) dan topeng segmentasi piksel biner (`boxMask`) untuk setiap rintangan yang terdeteksi (seperti Bus, Mobil, Motor, Orang, dan Truk). Untuk mengaitkan rintangan visual tersebut dengan jarak kedalaman fisiknya secara andal, dirancang sistem estimasi jarak adaptif tiga tingkat (*3-tier fallback distance estimator*) pada node `yolov8_seg_lidar_node` di dalam file `main.cpp`:

##### a. Strategi 1: Fusi Masker Segmentasi Instans (Instance Mask Fusion)
Metode ini merupakan pilihan utama (*primary strategy*) karena memiliki tingkat akurasi spasial tertinggi. Sistem memetakan seluruh koordinat piksel titik LiDAR terproyeksi $(u, v)$ yang jatuh di dalam batas kotak pembatas objek. Apabila koordinat lokal titik tersebut terhadap sudut kiri atas kotak pembatas $(x_{\text{bbox}}, y_{\text{bbox}})$ bernilai aktif pada masker segmentasi biner:
$$\text{boxMask}(v - y_{\text{bbox}}, \, u - x_{\text{bbox}}) > 0$$

Maka nilai jarak radial ($r_i$) dan arah sudut datang ($\theta_{\text{adj}, i}$) dari titik LiDAR tersebut akan disimpan. Jarak rata-rata ($d_{\text{avg}}$) dan sudut rata-rata ($\theta_{\text{avg}}$) dari objek dihitung dari $N_{\text{mask}}$ titik yang memenuhi syarat:
$$d_{\text{avg}} = \frac{1}{N_{\text{mask}}} \sum_{i=1}^{N_{\text{mask}}} r_i$$
$$\theta_{\text{avg}} = \frac{1}{N_{\text{mask}}} \sum_{i=1}^{N_{\text{mask}}} \theta_{\text{adj}, i}$$

Di mana:
*   $d_{\text{avg}}$ adalah estimasi jarak akhir rintangan hasil fusi masker (meter).
*   $\theta_{\text{avg}}$ adalah sudut datang rintangan hasil fusi masker (radian).
*   $N_{\text{mask}}$ adalah jumlah total titik LiDAR terproyeksi yang berada di dalam topeng segmentasi aktif objek.
*   $r_i$ dan $\theta_{\text{adj}, i}$ adalah nilai jarak (meter) dan sudut (radian) dari titik LiDAR terproyeksi ke-$i$.

##### b. Strategi 2: Pencarian Radius Terdekat (Nearby Radius Search)
Apabila masker segmentasi tidak memuat satu pun titik LiDAR terproyeksi (misalnya karena ketidaksempurnaan bentuk masker atau derau kalibrasi koordinat), sistem akan beralih ke strategi kedua. Algoritma menghitung pusat piksel kotak pembatas objek $(u_{\text{center}}, v_{\text{center}})$:
$$u_{\text{center}} = x_{\text{bbox}} + \frac{W_{\text{bbox}}}{2}, \quad v_{\text{center}} = y_{\text{bbox}} + \frac{H_{\text{bbox}}}{2}$$

Lalu mendefinisikan radius pencarian piksel dinamis ($R_{\text{search}}$) berdasarkan dimensi terbesar kotak pembatas objek ditambah offset toleransi:
$$R_{\text{search}} = \max\left(\frac{W_{\text{bbox}}}{2}, \, \frac{H_{\text{bbox}}}{2}\right) + 30\text{ piksel}$$

Sistem kemudian mencari satu titik LiDAR terproyeksi $j$ yang meminimalkan jarak piksel Euclidean terhadap pusat objek $(u_{\text{center}}, v_{\text{center}})$ dalam batas radius $R_{\text{search}}$:
$$j = \text{arg min}_{p} \left[ (u_p - u_{\text{center}})^2 + (v_p - v_{\text{center}})^2 \right] \quad \text{di mana } \text{jarak\_Euclidean} \le R_{\text{search}}$$

Jarak dan sudut objek langsung ditentukan berdasarkan data sensor titik $j$ tersebut: $d_{\text{avg}} = r_j$ dan $\theta_{\text{avg}} = \theta_j$.

##### c. Strategi 3: Model Kamera Lubang Jarum BBox Fallback (BBox Geometrical Estimation)
Jika sensor LiDAR mengalami kegagalan total, terhalang kotoran fisik, atau objek berada di luar jangkauan deteksi aktif LiDAR, sistem akan menggunakan strategi terakhir sebagai penyelamat (*worst-case fallback*). Jarak rintangan diestimasi secara geometris murni dari citra kamera menggunakan perbandingan lebar nyata objek ($W_{\text{real}}$) terhadap lebar piksel kotak deteksi ($W_{\text{bbox}}$) melalui model proyeksi kamera lubang jarum (*pinhole camera model*):
$$d_{\text{est}} = \frac{W_{\text{real}} \cdot f_x}{W_{\text{bbox}}}$$

Di mana:
*   $d_{\text{est}}$ adalah jarak estimasi rintangan berdasarkan geometri citra (meter).
*   $W_{\text{real}}$ adalah asumsi lebar fisik standar rintangan dalam meter, diambil dari konstanta basis data objek: Bus ($2.5\text{m}$), Car ($1.8\text{m}$), Motorcycle ($0.8\text{m}$), Person ($0.5\text{m}$), Truck ($2.3\text{m}$).
*   $f_x$ adalah panjang fokus lensa kamera horizontal ($939.988307\text{ px}$).
*   $W_{\text{bbox}}$ adalah lebar kotak pembatas objek terdeteksi dalam citra (piksel).

Sudut datang objek ($\theta_{\text{est}}$) dihitung secara geometris berdasarkan penyimpangan posisi horizontal pusat kotak pembatas terhadap pusat optik horizontal kamera ($c_x$):
$$\theta_{\text{est}} = \arctan\left(\frac{u_{\text{center}} - c_x}{f_x}\right)$$

##### d. Penentuan Objek Aktif untuk Kontrol Navigasi
Setelah seluruh rintangan memiliki nilai jarak dan sudut, sistem menyaring objek yang berada di dalam bidang pandang kritis depan (sudut $\theta_{\text{avg}} \in [-150^\circ, 150^\circ]$). Rintangan dengan jarak $d_{\text{avg}}$ terkecil dipilih sebagai **Objek Aktif Terdekat** dan ID kelasnya dipublikasikan ke topik `/active_object_type`. Informasi spasial dari seluruh rintangan lainnya digabungkan ke dalam pesan larik multi-dimensi topik `/sfm_context` untuk dikirimkan secara langsung ke unit perencana gerak *Social Force Model* (SFM) guna menghitung gaya repulsif longitudinal dan lateral robot.
