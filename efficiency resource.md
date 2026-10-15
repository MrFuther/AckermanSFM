Searched for "CMAKE_BUILD_TYPE"
Listed directory ackerman
Viewed CMakeLists.txt:1-278

Tentu saja, **masih banyak sekali ruang untuk optimasi** penggunaan CPU, Memory, dan GPU pada sistem yang dijalankan oleh `integration.launch` **tanpa harus mengubah logika kode program atau fungsionalitasnya**. 

Pada komputer papan tunggal seperti **Jetson Nano**, optimasi tingkat sistem dan kompilasi sering kali memberikan dampak performa yang sangat signifikan (bisa menghemat CPU hingga 30%-50%).

Berikut adalah beberapa cara optimasi tingkat sistem, konfigurasi, dan lingkungan (*environment*) yang dapat Anda terapkan langsung:

---

### 1. Optimasi Kompilasi C++ (Compiler Flags)
Secara default, jika Anda melakukan `catkin_make` biasa, ROS akan mengompilasi program dalam mode debug tanpa optimalisasi kode, sehingga kinerjanya sangat lambat.
* **Solusi**: Kompilasi ulang seluruh C++ nodes Anda menggunakan tipe **Release** dengan optimasi tingkat tinggi (`-O3`) dan optimasi arsitektur prosesor ARM (`-march=native` dan `-mfpu=neon`).
* **Langkah**: Jalankan perintah ini di root catkin workspace Anda:
  ```bash
  catkin_make -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-O3 -march=native -mfpu=neon"
  ```
  * **Hasil**: Kode biner C++ (seperti `lanekeepingassist`, `fuzzy`, `sfmazra`) akan berjalan jauh lebih cepat dan menggunakan CPU yang jauh lebih sedikit karena compiler akan mengaktifkan fitur *auto-vectorization* dan instruksi NEON pada prosesor ARM Jetson Nano.

---

### 2. Mengaktifkan Performa Maksimal Hardware Jetson (Nvidia Power Mode)
Jetson Nano memiliki profil manajemen daya dinamis yang secara default membatasi kecepatan CPU/GPU untuk menghemat daya.
* **Solusi**: Atur Jetson Nano ke mode daya penuh (10W / MAXN) dan kunci kecepatan clock CPU, GPU, serta Memory Bus ke titik tertingginya.
* **Langkah**: Jalankan perintah berikut di terminal Jetson Anda:
  ```bash
  # Mengaktifkan mode daya maksimum (MAXN)
  sudo nvpmodel -m 0

  # Mengunci clock CPU/GPU ke kecepatan maksimum
  sudo jetson_clocks
  ```
  * **Hasil**: Menghilangkan *thermal throttling* dan *latency spikes*. Program Anda akan berjalan jauh lebih konsisten dengan FPS yang stabil.

---

### 3. Mengurangi Latency USB Serial (Optimasi Kernel)
Node `serial_node` (`rosserial_server`) dan `rplidarNode` berkomunikasi melalui modul USB-to-Serial. Secara default di Linux, driver USB serial memiliki **latency timer sebesar 16 ms**. Ini memaksa sistem menunggu 16 ms sebelum mengirim data ke ROS, meningkatkan beban CPU akibat *polling thread*.
* **Solusi**: Turunkan latency timer USB menjadi **1 ms**.
* **Langkah**: Jalankan perintah berikut (sesuaikan port USB jika berbeda, misalnya `ttyUSB0` atau `ttyACM0`):
  ```bash
  sudo sh -c 'echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer'
  ```
  * **Hasil**: Mengurangi delay data LiDAR dan encoder secara drastis, serta menurunkan penggunaan CPU dari node serial karena tidak perlu melakukan *overhead polling* yang intens.

---

### 4. Nonaktifkan Output Terminal (`output="screen"`) untuk Node Background
Di file [integration.launch](file:///a:/ACKERMAN/ackerman/launch/integration.launch), beberapa node diatur dengan atribut `output="screen"`.
* **Analisis**: Menampilkan teks log secara terus-menerus ke layar terminal (stdout) menggunakan CPU yang cukup besar untuk rendering teks di antarmuka terminal.
* **Solusi**: Ubah node yang bekerja di latar belakang (seperti `rplidarNode`, `serial_node`, `encoder_read`) menjadi `output="log"`. Biarkan hanya node krusial yang menampilkan output ke layar.
  ```xml
  <node name="rplidarNode" pkg="rplidar_ros" type="rplidarNode" output="log">
  ```

---

### 5. Membatasi Threading pada OpenCV
Secara bawaan, library OpenCV yang digunakan di `lanekeepingassist` dan `dashboard` akan mencoba menggunakan semua core CPU yang tersedia (multithreading) untuk fungsi pengolahan citra sederhana. Pada prosesor 4-core seperti Jetson Nano, hal ini dapat menyebabkan *CPU thrashing* (perebutan core antarnode) yang membuat performa menurun.
* **Solusi**: Batasi jumlah thread OpenCV menjadi maksimal 2 thread per node melalui *environment variable*.
* **Langkah**: Masukkan baris berikut di terminal sebelum menjalankan `roslaunch`, atau tambahkan ke dalam file `.bashrc` Anda:
  ```bash
  export OPENCV_NUM_THREADS=2
  ```
  Atau masukkan langsung ke launch file Anda di bagian paling atas:
  ```xml
  <env name="OPENCV_NUM_THREADS" value="2" />
  ```

---

### 6. Mengurangi Beban Visualisasi Headless (GUI/Dashboard)
Pada `integration.launch`, terdapat node visualisasi:
* `<node name="animation" pkg="ackerman" type="animationnew.py" output="screen"/>`
* `<node pkg="ackerman" type="dashboard" name="dashboard" output="screen"/>`

Jika robot Anda sedang dalam mode pengujian jalan riil (tidak terhubung ke monitor fisik dan dijalankan secara mandiri/headless):
* **Solusi**: Matikan node GUI tersebut dengan memberikan komentar (`<!-- -->`) pada file launch:
  ```xml
  <!-- <node name="animation" pkg="ackerman" type="animationnew.py" output="screen"/> -->
  <!-- <node pkg="ackerman" type="dashboard" name="dashboard" output="screen"/> -->
  ```
  * **Hasil**: OpenCV GUI (`cv::imshow` / `cv::waitKey`) dan Matplotlib (`animationnew.py`) memakan resource GPU/Memory yang sangat besar. Mematikan visualisasi ini saat robot berjalan mandiri akan langsung **menghemat RAM ratusan MB** dan menghemat CPU secara signifikan.