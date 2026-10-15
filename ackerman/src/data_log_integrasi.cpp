#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h> // Tambah include untuk Int32
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <mutex>
#include <deque>

// Variabel Global untuk menyimpan data terbaru
std::mutex log_mutex;
double current_fnav = 0.0;
double current_distance = 0.0;
double current_vel_cmd = 0.0;     // Velocity Command (m/s)
double current_real_vel = 0.0;    // Real Velocity (m/s)
int current_vt = 0;               // Velocity Vt (PWM/Int)
int current_active_object = -1;   // ID Objek Aktif (-1 = No Object)
double current_lane_error = 0.0;  // Simpangan lajur dari LKA
double lka_instant_fps = 0.0;     // FPS instan LKA
double lka_avg_fps = 0.0;         // FPS rata-rata LKA
bool is_autonomous_speed = false; // Status otonom kecepatan (tombol T)
bool is_autonomous_steer = false; // Status otonom kemudi (tombol Y)
std::deque<ros::Time> lka_timestamps;
ros::Time last_lka_recv_time(0.0);

// Variabel tambahan untuk eksperimen ADAS Ackerman
double current_rho = 180.0;       // Sudut Kemudi (derajat) - default lurus (180.0)
double current_v0_target = 0.3;   // Target kecepatan v0 (m/s)
double current_f_obs_y = 0.0;     // Gaya tolak lateral objek (N)
double current_f_lane_y = 0.0;    // Gaya koreksi lateral lajur (N)

// --- CALLBACKS ---

// 1. Ambil Fnav (Magnitude) dari sfmazra.cpp
void fnavCallback(const geometry_msgs::Point::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    current_fnav = msg->x;
}

// 2. Ambil Distance (m) dari sfmazra.cpp
void distanceCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    if (!msg->data.empty()) {
        current_distance = msg->data[0];
    }
}

// 3. Ambil Velocity Command (m/s) dari drive.cpp
void veloCmdCallback(const std_msgs::Float64::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    current_vel_cmd = msg->data;
}

// 4. Ambil Velocity Vt (Output PWM/Int) dari drive.cpp
void vtCallback(const std_msgs::UInt16::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    current_vt = msg->data;
}

// 5. Ambil Real Velocity (m/s) dari sensor/encoder
void realSpeedCallback(const std_msgs::Float64::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    current_real_vel = msg->data;
}

// 6. Ambil ID Objek Aktif dari sfmazra.cpp
void activeObjectCallback(const std_msgs::Int32::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    current_active_object = msg->data;
}

// 7. Ambil Lane Error (px) dari LKA
void laneErrorCallback(const std_msgs::Float32::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    current_lane_error = msg->data;
    
    ros::Time current_time = ros::Time::now();
    if (!last_lka_recv_time.isZero()) {
        double dt = (current_time - last_lka_recv_time).toSec();
        if (dt > 0.0) {
            lka_instant_fps = 1.0 / dt;
        }
    }
    last_lka_recv_time = current_time;
    lka_timestamps.push_back(current_time);
    if (lka_timestamps.size() > 10) {
        lka_timestamps.pop_front();
    }
    if (lka_timestamps.size() >= 2) {
        double total_dt = (lka_timestamps.back() - lka_timestamps.front()).toSec();
        if (total_dt > 0.0) {
            lka_avg_fps = (lka_timestamps.size() - 1) / total_dt;
        }
    }
}

// 8. Ambil status kontrol dari keyboard_handler
void controlArrayCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    if (msg->data.size() >= 4) {
        is_autonomous_speed = (msg->data[2] == 1.0f);
        is_autonomous_steer = (msg->data[3] == 1.0f);
    }
}

// 9. Ambil Sudut Kemudi (Rho) dari sfmazra.cpp
void rhoCallback(const geometry_msgs::Point::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    current_rho = msg->x;
}

// 10. Ambil Kecepatan Target v0 dari driveazra.cpp
void v0TargetCallback(const std_msgs::Float64::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    current_v0_target = msg->data;
}

// 11. Ambil Gaya Tolak Lateral Objek dari sfmazra.cpp
void fObsYCallback(const std_msgs::Float32::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    current_f_obs_y = msg->data;
}

// 12. Ambil Gaya Koreksi Lateral Lajur dari sfmazra.cpp
void fLaneYCallback(const std_msgs::Float32::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    current_f_lane_y = msg->data;
}

// Fungsi helper untuk timestamp di dalam kolom CSV
std::string getROSTimestamp() {
    ros::Time now = ros::Time::now();
    std::time_t seconds = now.sec;
    std::tm* time_info = std::localtime(&seconds);
    std::ostringstream timestamp;
    timestamp << std::put_time(time_info, "%H:%M:%S");
    timestamp << "." << std::setw(3) << std::setfill('0') << (now.nsec / 1000000);
    return timestamp.str();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_data_logger");
    ros::NodeHandle nh;

    // --- PEMBUATAN NAMA FILE ---
    std::time_t now = std::time(nullptr);
    char filename_buffer[100];
    std::strftime(filename_buffer, sizeof(filename_buffer), "data_integrasi_%H_%d-%m-%Y.csv", std::localtime(&now));
    std::string filename = filename_buffer;

    std::ofstream log_file;
    log_file.open(filename, std::ios::out);

    if (!log_file.is_open()) {
        ROS_FATAL("Gagal membuat file log: %s", filename.c_str());
        return -1;
    }

    ROS_INFO("Memulai logging ke file: %s", filename.c_str());

    // Tulis Header CSV
    log_file << "Timestamp,"
             << "Velocity Vt (PWM),"
             << "Fnav (Force),"
             << "Distance (m),"
             << "Velocity Command (m/s),"
             << "Real Velocity (m/s),"
             << "Active Object ID,"
             << "Lane Error (px),"
             << "LKA Instant FPS,"
             << "LKA Avg FPS,"
             << "Steering Angle (Rho deg),"
             << "V0 Target (m/s),"
             << "F_obs_y (N),"
             << "F_lane_y (N)\n";

    // --- SUBSCRIBERS ---
    ros::Subscriber sub_fnav = nh.subscribe("/fnav", 10, fnavCallback);
    ros::Subscriber sub_dist = nh.subscribe("/near_distance", 10, distanceCallback);
    ros::Subscriber sub_cmd  = nh.subscribe("/velopub", 10, veloCmdCallback);
    ros::Subscriber sub_vt   = nh.subscribe("v1", 10, vtCallback);
    ros::Subscriber sub_real = nh.subscribe("/vehicle_speed", 10, realSpeedCallback);
    ros::Subscriber sub_active_object = nh.subscribe("/active_object_type", 10, activeObjectCallback); // Subscriber baru
    ros::Subscriber sub_lane_error = nh.subscribe("/lane_error", 10, laneErrorCallback);
    ros::Subscriber sub_control = nh.subscribe("/control_array", 10, controlArrayCallback);
    ros::Subscriber sub_rho = nh.subscribe("/rho", 10, rhoCallback);
    ros::Subscriber sub_v0_target = nh.subscribe("/v0_target", 10, v0TargetCallback);
    ros::Subscriber sub_f_obs_y = nh.subscribe("/sfm/f_obs_y", 10, fObsYCallback);
    ros::Subscriber sub_f_lane_y = nh.subscribe("/sfm/f_lane_y", 10, fLaneYCallback);

    ros::Rate rate(30); // 30 Hz logging rate

    bool was_logging = false;
    while (ros::ok()) {
        ros::spinOnce();
        
        {
            std::lock_guard<std::mutex> lock(log_mutex);
            
            // Log data hanya jika robot berjalan otonom (tombol T dan Y keduanya aktif)
            if (is_autonomous_speed && is_autonomous_steer) {
                if (!was_logging) {
                    ROS_INFO("Robot berjalan otonom. Mulai menulis data log integrasi...");
                    was_logging = true;
                }
                // Tulis data ke file
                log_file << getROSTimestamp() << ","
                         << current_vt << ","
                         << current_fnav << ","
                         << current_distance << ","
                         << current_vel_cmd << ","
                         << current_real_vel << ","
                         << current_active_object << ","
                         << current_lane_error << ","
                         << lka_instant_fps << ","
                         << lka_avg_fps << ","
                         << current_rho << ","
                         << current_v0_target << ","
                         << current_f_obs_y << ","
                         << current_f_lane_y << "\n";
                log_file.flush();
            } else {
                if (was_logging) {
                    ROS_INFO("Robot keluar dari mode otonom. Menangguhkan penulisan data log...");
                    was_logging = false;
                }
            }
        }

        rate.sleep();
    }

    log_file.close();
    ROS_INFO("Logging selesai. File disimpan.");
    return 0;
}