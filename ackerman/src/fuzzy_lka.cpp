#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <algorithm>

class FuzzyLKANode {
private:
    ros::NodeHandle nh;
    ros::Subscriber sub_error;
    ros::Publisher pub_flane;

    float prev_error;
    ros::Time last_time;

    // ==========================================
    // 1. FUNGSI KEANGGOTAAN (MEMBERSHIP FUNCTIONS)
    // ==========================================
    // Fungsi Segitiga (Tengah)
    float trimf(float x, float a, float b, float c) {
        if (x <= a || x >= c) return 0.0;
        if (x == b) return 1.0;
        if (x < b) return (x - a) / (b - a);
        return (c - x) / (c - b);
    }

    // Fungsi Trapesium (Tepi Kiri/Bawah)
    float trapmf_left(float x, float a, float b) {
        if (x <= a) return 1.0;
        if (x >= b) return 0.0;
        return (b - x) / (b - a);
    }

    // Fungsi Trapesium (Tepi Kanan/Atas)
    float trapmf_right(float x, float a, float b) {
        if (x <= a) return 0.0;
        if (x >= b) return 1.0;
        return (x - a) / (b - a);
    }

public:
    FuzzyLKANode() {
        // Menerima raw pixel error dari lanekeepingassist.cpp
        sub_error = nh.subscribe("/lane_error", 10, &FuzzyLKANode::errorCallback, this);
        // Mengirimkan gaya F_lane yang sudah halus ke sfmazra.cpp
        pub_flane = nh.advertise<std_msgs::Float32>("/f_lane_fuzzy", 10);
        
        prev_error = 0.0;
        last_time = ros::Time::now();
    }

    void errorCallback(const std_msgs::Float32::ConstPtr& msg) {
        float error = msg->data; 
        ros::Time current_time = ros::Time::now();
        float dt = (current_time - last_time).toSec();
        if (dt <= 0) dt = 0.033; // Cegah error pembagian nol (asumsi ~30 FPS)

        // Delta Error: Mengetahui seberapa cepat robot menyimpang
        float delta_error = (error - prev_error) / dt;

        // ==========================================
        // 2. FUZZIFIKASI (Menerjemahkan angka ke bahasa linguistik)
        // ==========================================
        // Himpunan Error (Piksel): Asumsi layar 640. Melenceng max ~320.
        float e_KiriJauh = trapmf_left(error, -150, -50);
        float e_Kiri     = trimf(error, -100, -50, 0);
        float e_Tengah   = trimf(error, -30, 0, 30);
        float e_Kanan    = trimf(error, 0, 50, 100);
        float e_KananJauh= trapmf_right(error, 50, 150);

        // Himpunan Delta Error: 
        // N (Negatif/Menjauh ke kiri), Z (Stabil), P (Positif/Menjauh ke kanan)
        float de_Negatif = trapmf_left(delta_error, -50, -10);
        float de_Stabil  = trimf(delta_error, -20, 0, 20);
        float de_Positif = trapmf_right(delta_error, 10, 50);

        // ==========================================
        // 3. EVALUASI ATURAN (RULE BASE) & DEFUZZIFIKASI SUGENO
        // ==========================================
        float out_num = 0.0; // Pembilang
        float out_den = 0.0; // Penyebut

        // Lambda function untuk menghitung bobot setiap rule (Metode Center of Gravity)
        auto add_rule = [&](float w_error, float w_delta, float gaya_output) {
            float bobot = std::min(w_error, w_delta); // Logika AND
            out_num += bobot * gaya_output;
            out_den += bobot;
        };

        // Kategori Output Gaya (F_lane Normalisasi: -1.0 sampai 1.0)
        float Tarik_Kiri_Kuat = -1.0;
        float Tarik_Kiri_Halus = -0.5;
        float Lurus = 0.0;
        float Tarik_Kanan_Halus = 0.5;
        float Tarik_Kanan_Kuat = 1.0;

        // MATRIX RULES (Kecerdasan Buatan)
        // Kasus 1: Robot di Kiri Jauh
        add_rule(e_KiriJauh, de_Negatif, Tarik_Kanan_Kuat);  // Di kiri dan makin menjauh! Tarik kuat!
        add_rule(e_KiriJauh, de_Stabil, Tarik_Kanan_Kuat);
        add_rule(e_KiriJauh, de_Positif, Tarik_Kanan_Halus); // Di kiri tapi sedang proses balik ke tengah. Jangan tarik kuat agar tidak melintir (overshoot).

        // Kasus 2: Robot di Kiri Sedikit
        add_rule(e_Kiri, de_Negatif, Tarik_Kanan_Kuat);
        add_rule(e_Kiri, de_Stabil, Tarik_Kanan_Halus);
        add_rule(e_Kiri, de_Positif, Lurus); 

        // Kasus 3: Robot di Tengah (Aman)
        add_rule(e_Tengah, de_Negatif, Tarik_Kanan_Halus); // Di tengah tapi hidung mulai goyang ke kiri
        add_rule(e_Tengah, de_Stabil, Lurus);              // Perfect!
        add_rule(e_Tengah, de_Positif, Tarik_Kiri_Halus);  // Di tengah tapi hidung mulai goyang ke kanan

        // Kasus 4: Robot di Kanan Sedikit
        add_rule(e_Kanan, de_Negatif, Lurus);
        add_rule(e_Kanan, de_Stabil, Tarik_Kiri_Halus);
        add_rule(e_Kanan, de_Positif, Tarik_Kiri_Kuat);

        // Kasus 5: Robot di Kanan Jauh
        add_rule(e_KananJauh, de_Negatif, Tarik_Kiri_Halus); // Sedang proses balik ke tengah, kalem saja.
        add_rule(e_KananJauh, de_Stabil, Tarik_Kiri_Kuat);
        add_rule(e_KananJauh, de_Positif, Tarik_Kiri_Kuat);  // Di kanan dan makin menjauh! Tarik kuat!

        // Menghitung Hasil Akhir Gaya (Weighted Average)
        float f_lane_final = (out_den == 0) ? 0.0 : (out_num / out_den);

        // ==========================================
        // 4. PUBLISH HASIL
        // ==========================================
        std_msgs::Float32 msg_out;
        msg_out.data = f_lane_final; 
        pub_flane.publish(msg_out);

        // Simpan data untuk siklus berikutnya
        prev_error = error;
        last_time = current_time;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fuzzy_lka_node");
    FuzzyLKANode fuzzy_node;
    ROS_INFO("Fuzzy Logic Node Active. Listening to /lane_error, Publishing /f_lane_fuzzy");
    ros::spin();
    return 0;
}