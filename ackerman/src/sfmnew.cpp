#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32.h"
#include <vector>
#include <cmath>
#include <numeric>

// --- Parameter yang bisa di-tuning ---
const double SAFE_FOLLOWING_DISTANCE = 1.5; // Jarak aman dari target (meter)
const double ATTRACTIVE_FORCE_GAIN = 1.0;   // Seberapa kuat tarikan ke target

// Publisher global
ros::Publisher magnitude_publisher;
ros::Publisher direction_publisher;
ros::Publisher vector_y_publisher;
ros::Publisher vector_x_publisher;
ros::Publisher near_distance_publisher;

// Variabel global untuk menyimpan data dari subscriber
geometry_msgs::Point current_target;
std::vector<std::pair<float, float>> other_obstacles;
bool target_received = false;
float k = 0; // Gain dari fuzzy logic

// === CALLBACKS UNTUK MENERIMA DATA ===

void targetCallback(const geometry_msgs::Point::ConstPtr& msg) {
    current_target = *msg;
    target_received = true;
}

void obstaclesCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    other_obstacles.clear();
    for (size_t i = 0; i < msg->data.size(); i += 2) {
        other_obstacles.emplace_back(msg->data[i], msg->data[i+1]);
    }
}

void fuzzyCallback(const std_msgs::Float32::ConstPtr& msg){
    k = msg->data;
}

// === FUNGSI PERHITUNGAN GAYA ===

float calculateCoordinates(float degree, float a = 1, float b = 0.5) {
    float angle = degree * M_PI / 180.0;
    float r = (a * b) / std::sqrt(std::pow(b * std::cos(angle), 2) + std::pow(a * std::sin(angle), 2));
    return r;
}

// Gaya tolakan (Repulsive Force) - TIDAK BERUBAH
float forceSocial(float distance, float degree) {
    float psi = 0.35;
    float r_value = calculateCoordinates(degree);
    return k * std::exp((r_value - distance) / psi);
}

float forcePhysical(float distance, float degree) {
    float r_value = calculateCoordinates(degree);
    return k * (r_value - distance);
}

// (BARU) Gaya tarikan (Attractive Force)
float forceAttraction(float target_distance) {
    // Gaya ini seperti pegas: menarik jika jauh, mendorong jika terlalu dekat
    return -ATTRACTIVE_FORCE_GAIN * (target_distance - SAFE_FOLLOWING_DISTANCE);
}


// === LOGIKA UTAMA SFM ===

void sfm() {
    float resultant_vector_x = 0.0;
    float resultant_vector_y = 0.0;
    
    // --- Langkah 1: Hitung Gaya Tarikan ke Target ---
    if (target_received) {
        float target_dist = current_target.x;
        float target_angle_deg = current_target.y;
        float target_angle_rad = target_angle_deg * M_PI / 180.0;

        float f_attraction = forceAttraction(target_dist);

        // Tambahkan komponen gaya tarikan ke resultan
        resultant_vector_x += f_attraction * std::cos(target_angle_rad);
        resultant_vector_y += f_attraction * std::sin(target_angle_rad);
    }
    
    // --- Langkah 2: Hitung Gaya Tolakan dari Semua Rintangan Lain ---
    float nearest_obstacle_dist = 1e9; // Untuk dikirim ke fuzzy

    for (const auto& obstacle : other_obstacles) {
        float obstacle_dist = obstacle.first;
        float obstacle_angle_deg = obstacle.second;

        if (obstacle_dist < nearest_obstacle_dist) {
            nearest_obstacle_dist = obstacle_dist;
        }

        // Cek tumpang tindih (overlap) elips untuk setiap rintangan
        float r_robot = calculateCoordinates(obstacle_angle_deg);
        float r_obstacle = calculateCoordinates(obstacle_angle_deg + 180);
        float effective_distance = obstacle_dist - r_obstacle;
        
        if (effective_distance < r_robot) { // Jika overlap
            float f_social = forceSocial(effective_distance, obstacle_angle_deg);
            float f_physical = forcePhysical(effective_distance, obstacle_angle_deg);
            float f_repulsion_total = f_social + f_physical;

            // Arah gaya tolakan adalah kebalikan dari arah rintangan
            float repulsion_angle_rad = (obstacle_angle_deg + 180.0) * M_PI / 180.0;

            // Tambahkan komponen gaya tolakan ke resultan
            resultant_vector_x += f_repulsion_total * std::cos(repulsion_angle_rad);
            resultant_vector_y += f_repulsion_total * std::sin(repulsion_angle_rad);
        }
    }
    
    // Publikasikan jarak rintangan terdekat untuk digunakan oleh fuzzy node
    std_msgs::Float32 dmsg;
    dmsg.data = (nearest_obstacle_dist > 10) ? 5.0 : nearest_obstacle_dist; // Kirim nilai default jika tidak ada rintangan
    near_distance_publisher.publish(dmsg);

    // --- Langkah 3: Hitung Vektor Resultan Akhir dan Publikasikan ---
    float resultant_magnitude = std::sqrt(resultant_vector_x * resultant_vector_x + resultant_vector_y * resultant_vector_y);
    float resultant_angle_rad = std::atan2(resultant_vector_y, resultant_vector_x);
    float resultant_angle_deg = resultant_angle_rad * 180.0 / M_PI;

    // Publikasikan komponen X dan Y
    std_msgs::Float32 xmsg;
    xmsg.data = resultant_vector_x;
    vector_x_publisher.publish(xmsg);

    std_msgs::Float32 ymsg;
    ymsg.data = resultant_vector_y;
    vector_y_publisher.publish(ymsg);

    // Publikasikan Magnitudo (fnav)
    geometry_msgs::Point magnitude_msg;
    magnitude_msg.x = resultant_magnitude;
    magnitude_publisher.publish(magnitude_msg);

    // Publikasikan Arah (rho)
    geometry_msgs::Point direction_msg;
    direction_msg.x = resultant_angle_deg;
    direction_publisher.publish(direction_msg);
}

// === MAIN FUNCTION ===

int main(int argc, char **argv) {
    ros::init(argc, argv, "sfm_node");
    ros::NodeHandle n;

    // Inisialisasi Publisher
    magnitude_publisher = n.advertise<geometry_msgs::Point>("/fnav", 10);
    direction_publisher = n.advertise<geometry_msgs::Point>("/rho", 10);
    vector_y_publisher = n.advertise<std_msgs::Float32>("/resultant_vector_y", 10);
    vector_x_publisher = n.advertise<std_msgs::Float32>("/resultant_vector_x", 10);
    near_distance_publisher = n.advertise<std_msgs::Float32>("/near_distance", 10);

    // Inisialisasi Subscriber (BARU)
    ros::Subscriber sub_target = n.subscribe("/target_object", 10, targetCallback);
    ros::Subscriber sub_obstacles = n.subscribe("/other_obstacles", 10, obstaclesCallback);
    ros::Subscriber sub_fuzzy = n.subscribe("/fuzzy", 10, fuzzyCallback);

    ros::Rate loop_rate(30);
    while (ros::ok()) {
        sfm();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}