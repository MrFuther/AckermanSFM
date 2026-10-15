#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <utility>
#include <deque> 
#include "std_msgs/Float64.h"

// Precalculated constants for optimization
const double DEG_TO_RAD = M_PI / 180.0;
const double RAD_TO_DEG = 180.0 / M_PI;

ros::Publisher magnitude_publisher;
ros::Publisher direction_publisher;
ros::Publisher vector_y_publisher;
ros::Publisher vector_x_publisher;
ros::Publisher near_distance;
ros::Publisher stable_distance_publisher;
ros::Publisher distance_filtered_publisher;
ros::Publisher a_dyn_publisher;

std::vector<std::pair<float, float>> distances_minus150_to_150;

float current_vehicle_speed = 0.0f;
float current_obstacle_speed = 0.0f;
float current_v0 = 1.6f; // Nilai default v0 dari keyboard_handler
float velocity_gain = 1.0f; // K_v: penambah area proksemik berbasis kecepatan relatif
float dynamic_a_min = 0.5f;
float dynamic_a_max = 5.0f;

// (MODIFIKASI) State variables untuk filter dan delta_distance
static std::deque<double> history_jarak; 
static double history_jarak_sum = 0.0;
const size_t filter_size = 8; //8 aslinya
static double jarak_sebelumnya_terfilter = 0.0; 
static bool sdh_diinisialisasi = false;
static const double dt = 0.1;
static double d_stable = 0.0;
static bool d_initialized = false;
static double d_stable_sebelumnya = 0.0;
static bool d_delta_initialized = false;

float resultant_magnitude = 0.0;
float resultant_angle = 0.0;
float k = 0.0; // Default fuzzy gain
float f_req = 0.0f; // F request dari topik /f_req
float psi = 0.55f;

void distancesMinus150to150Callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    distances_minus150_to_150.clear();
    distances_minus150_to_150.reserve(msg->data.size() / 2);

    if (msg->data.size() % 2 != 0) {
        ROS_WARN("Received data size for distances_minus150_to_150 is not even. Skipping update.");
        return;
    }

    for (size_t i = 0; i < msg->data.size(); i += 2) {
        float distance = msg->data[i];
        float angle = msg->data[i + 1];
        distances_minus150_to_150.emplace_back(distance, angle);
    }
}

void fuzzy_callback(const std_msgs::Float32::ConstPtr& msg){
    k = msg->data;
    ROS_DEBUG("SFM received fuzzy gain = %f", k);
}

void vehicleSpeedCallback(const std_msgs::Float64::ConstPtr& msg) {
    current_vehicle_speed = static_cast<float>(msg->data);
    if (current_vehicle_speed < 0) {
        current_vehicle_speed = 0; // Pastikan tidak negatif
    }
}

void obstacleSpeedCallback(const std_msgs::Float32::ConstPtr& msg) {
    current_obstacle_speed = std::max(0.0f, static_cast<float>(msg->data));
}

void v0Callback(const std_msgs::Float64::ConstPtr& msg) {
    // Gunakan 0.1 sebagai batas minimum untuk menghindari pembagian dengan nol
    current_v0 = (msg->data > 0.1) ? static_cast<float>(msg->data) : 0.1f;
}

void FreqCallback(const std_msgs::Float64::ConstPtr& msg) {
    f_req = static_cast<float>(msg->data);
    ROS_DEBUG("SFM received F_req = %f", f_req);
}

float overlapCalculated() {
    if (k != 0.0f && (f_req / k) > 0.0f) {
        return psi * std::log(f_req / k);
    }

    return 0.0f;
}

float calculateDynamicA() {
    const float d_ref = 1.5f;
    const float r_obs = 1.5f;

    float O_req = overlapCalculated();
    float v_rel = std::max(0.0f, current_vehicle_speed - current_obstacle_speed);
    float a_dyn = d_ref + O_req - r_obs + (velocity_gain * v_rel);
    if (a_dyn < dynamic_a_min) {
        a_dyn = dynamic_a_min;
    } else if (a_dyn > dynamic_a_max) {
        a_dyn = dynamic_a_max;
    }

    return a_dyn;
}

float calculateStaticCoordinates(float degree, float a = 1.5, float b = 0.5) {
    float angle = degree * DEG_TO_RAD;
    float cos_term = b * std::cos(angle);
    float sin_term = a * std::sin(angle);
    float r = (a * b) / std::sqrt((cos_term * cos_term) + (sin_term * sin_term));
    return r;
}

float calculateDynamicCoordinates(float degree, float a_dyn) {
    const float b0 = 0.5f;    // Sumbu semi-minor tetap

    // Rumus elips
    float angle = degree * DEG_TO_RAD;
    float cos_term = b0 * std::cos(angle);
    float sin_term = a_dyn * std::sin(angle);
    float r = (a_dyn * b0) / std::sqrt((cos_term * cos_term) + (sin_term * sin_term));
    return r;
}

float forceSocial(float distance, float r_value) {
    float force = k * std::exp((r_value - distance) / psi);

    return force;
}

float forcePhysical(float distance, float r_value) {
    float force = k * (r_value - distance);

    return force;
}

void sfm() {
    float nearest_distance_raw = 0.0; // Default large distance
    float nearest_angle = 0.0;

    bool isOverlap = false;

    auto nearest_minus150_to_150 = std::min_element(
        distances_minus150_to_150.begin(), distances_minus150_to_150.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; } 
    );

    if (nearest_minus150_to_150 == distances_minus150_to_150.end()) {
        ROS_DEBUG("SFM: No obstacle data available");
        nearest_distance_raw = 10.0;
        nearest_angle = 180.0;

        // Reset filter
        history_jarak_sum = 0.0;
        history_jarak.clear();
        sdh_diinisialisasi = false;
    }
    else{
        nearest_distance_raw = nearest_minus150_to_150->first;
        nearest_angle = nearest_minus150_to_150->second;
    }

    // --- (BLOK FILTER BARU) ---
    // Terapkan Filter Moving Average
    double jarak_mentah = static_cast<double>(nearest_distance_raw);
    history_jarak.push_back(jarak_mentah);
    history_jarak_sum += jarak_mentah;
    if (history_jarak.size() > filter_size) {
        history_jarak_sum -= history_jarak.front();
        history_jarak.pop_front();
    }

    double jarak_terfilter = 0.0;
    if (!history_jarak.empty()) {
        jarak_terfilter = history_jarak_sum / history_jarak.size();
    } else {
        jarak_terfilter = jarak_mentah;
    }

    // --- (BLOK DELTA_DISTANCE BARU) ---
    double jarak_sekarang_terfilter = jarak_terfilter;
    double delta_distance = 0.0;

    std_msgs::Float32 filtered_distance_msg;
    filtered_distance_msg.data = static_cast<float>(jarak_sekarang_terfilter);
    distance_filtered_publisher.publish(filtered_distance_msg);

    // Hanya hitung delta jika filter sudah "penuh"
    if (sdh_diinisialisasi && history_jarak.size() >= filter_size) {
        delta_distance = (jarak_sekarang_terfilter - jarak_sebelumnya_terfilter) / dt;
    }
    
    jarak_sebelumnya_terfilter = jarak_sekarang_terfilter; // Selalu update

    // Set flag inisialisasi hanya jika filter sudah penuh
    if (history_jarak.size() >= filter_size) {
        sdh_diinisialisasi = true;
    } else {
        sdh_diinisialisasi = false; // Belum siap
    }
    // --- (AKHIR BLOK DELTA_DISTANCE) ---

    //LIMIT LIDAR KE Drk
    double d_input = jarak_sekarang_terfilter;
    const double max_change = 0.025;
    
    if (!d_initialized){
    	d_stable = d_input;
    	d_initialized = true;
    }

    double selisih = d_input - d_stable;
    if (selisih > max_change) {
        selisih = max_change;
    }
    d_stable += selisih;

    // Delta distance untuk fuzzy berbasis d_stable
    if (d_delta_initialized) {
        delta_distance = (d_stable - d_stable_sebelumnya) / dt;
    } else {
        delta_distance = 0.0;
        d_delta_initialized = true;
    }
    d_stable_sebelumnya = d_stable;

    

    std_msgs::Float32 stable_distance_msg;
    stable_distance_msg.data = static_cast<float>(d_stable);
    stable_distance_publisher.publish(stable_distance_msg);

    std_msgs::Float32 a_dyn_msg;
    float a_dyn = calculateDynamicA();
    a_dyn_msg.data = a_dyn;
    a_dyn_publisher.publish(a_dyn_msg);

    // Publish nearest distance and delta_distance for fuzzy controller
    static std_msgs::Float32MultiArray dmsg;
    if (dmsg.data.capacity() < 2) {
        dmsg.data.reserve(2);
    }
    dmsg.data.clear();
    dmsg.data.push_back(static_cast<float>(d_stable));
    dmsg.data.push_back(static_cast<float>(delta_distance));
    near_distance.publish(dmsg);

    float r_robot = calculateDynamicCoordinates(nearest_angle, a_dyn);  //dynamic        
    float r_obstacle = calculateStaticCoordinates(nearest_angle + 180); 
        
    float effective_distance = d_stable - r_obstacle;
    if (effective_distance < r_robot) {
        isOverlap = true;
    }    

    // Calculate social force always (controlled by fuzzy gain k)
    float f_social = forceSocial(effective_distance, r_robot);
    
    // Initialize physical force
    float f_physical = 0.0;
    
    // Calculate physical force only when overlap occurs (emergency brake)
    if (isOverlap == true) {
        f_physical = forcePhysical(effective_distance, r_robot);
    }
    
    // Combine forces: social force (always) + physical force (only when overlap)
    float f_force_static = f_social + f_physical;
    float f_angle_reverse = nearest_angle * -1;

    float f_angle_reverse_rad = f_angle_reverse * DEG_TO_RAD;
    float resultant_vector_x = f_force_static * std::cos(f_angle_reverse_rad);
    float resultant_vector_y = f_force_static * std::sin(f_angle_reverse_rad);
    resultant_magnitude = sqrt(resultant_vector_x * resultant_vector_x + resultant_vector_y * resultant_vector_y);
    resultant_angle = atan2(resultant_vector_y, resultant_vector_x);
    resultant_angle = resultant_angle * RAD_TO_DEG;

    // Publish resultant vector components
    std_msgs::Float32 xmsg;
    xmsg.data = resultant_vector_x;
    
    vector_x_publisher.publish(xmsg);

    std_msgs::Float32 ymsg;
    ymsg.data = resultant_vector_y;
    vector_y_publisher.publish(ymsg);

    // Publish magnitude for drive controller
    geometry_msgs::Point magnitude_msg;
    magnitude_msg.x = resultant_magnitude;
    magnitude_msg.y = 0;
    magnitude_msg.z = 0;
    magnitude_publisher.publish(magnitude_msg);

    // Publish direction for drive controller
    geometry_msgs::Point direction_msg;
    direction_msg.x = resultant_angle;
    direction_msg.y = 0;
    direction_msg.z = 0;
    direction_publisher.publish(direction_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sfm_node");
    ros::NodeHandle n;
    ros::NodeHandle private_nh("~");

    private_nh.param("velocity_gain", velocity_gain, velocity_gain);
    private_nh.param("dynamic_a_min", dynamic_a_min, dynamic_a_min);
    private_nh.param("dynamic_a_max", dynamic_a_max, dynamic_a_max);

    // Publishers - Output to Drive and Fuzzy controllers
    magnitude_publisher = n.advertise<geometry_msgs::Point>("/fnav", 10);
    direction_publisher = n.advertise<geometry_msgs::Point>("/rho", 10);
    vector_y_publisher = n.advertise<std_msgs::Float32>("/resultant_vector_y", 10);
    vector_x_publisher = n.advertise<std_msgs::Float32>("/resultant_vector_x", 10);
    near_distance = n.advertise<std_msgs::Float32MultiArray>("/near_distance", 10);
    stable_distance_publisher = n.advertise<std_msgs::Float32>("/d_stable", 10);
    distance_filtered_publisher = n.advertise<std_msgs::Float32>("/distance_filtered", 10);
    a_dyn_publisher = n.advertise<std_msgs::Float32>("/a_dyn", 10);

    // Subscribers - Input from Lidar Scan and Fuzzy controllers
    ros::Subscriber sub_distances_minus150_to_150 = n.subscribe("/distances_minus150_to_150", 10, distancesMinus150to150Callback);
    ros::Subscriber sub_fuzzy = n.subscribe("/fuzzy", 10, fuzzy_callback);
    ros::Subscriber sub_speed = n.subscribe("/velopub", 10, vehicleSpeedCallback);
    ros::Subscriber sub_obstacle_speed = n.subscribe("/obstacle_speed_from_lidar", 10, obstacleSpeedCallback);
    ros::Subscriber sub_v0 = n.subscribe("/v0_target", 10, v0Callback);
    ros::Subscriber sub_f_req = n.subscribe("/f_req", 10, FreqCallback);

    ros::Rate loop_rate(30);
    while (ros::ok()) {
        sfm();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


// #include "ros/ros.h"
// #include "std_msgs/Float32MultiArray.h"
// #include "geometry_msgs/Point.h"
// #include "std_msgs/Float32.h"
// #include <iostream>
// #include <vector>
// #include <cmath>
// #include <numeric>
// #include <algorithm>
// #include <utility>
// #include <deque> 
// #include "std_msgs/Float64.h"

// // Precalculated constants for optimization
// const double DEG_TO_RAD = M_PI / 180.0;
// const double RAD_TO_DEG = 180.0 / M_PI;

// ros::Publisher magnitude_publisher;
// ros::Publisher direction_publisher;
// ros::Publisher vector_y_publisher;
// ros::Publisher vector_x_publisher;
// ros::Publisher near_distance;
// ros::Publisher stable_distance_publisher;
// ros::Publisher distance_filtered_publisher;

// std::vector<std::pair<float, float>> distances_minus150_to_150;

// float current_vehicle_speed = 0.0f;
// float current_v0 = 1.6f; // Nilai default v0 dari keyboard_handler

// // (MODIFIKASI) State variables untuk filter dan delta_distance
// static std::deque<double> history_jarak; 
// const size_t filter_size = 8; //8 aslinya
// static double jarak_sebelumnya_terfilter = 0.0; 
// static bool sdh_diinisialisasi = false;
// static const double dt = 0.1;
// static double d_stable = 0.0;
// static bool d_initialized = false;
// static double d_stable_sebelumnya = 0.0;
// static bool d_delta_initialized = false;

// float resultant_magnitude = 0.0;
// float resultant_angle = 0.0;
// float k = 0.0; // Default fuzzy gain

// void distancesMinus150to150Callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
//     distances_minus150_to_150.clear();

//     if (msg->data.size() % 2 != 0) {
//         ROS_WARN("Received data size for distances_minus150_to_150 is not even. Skipping update.");
//         return;
//     }

//     for (size_t i = 0; i < msg->data.size(); i += 2) {
//         float distance = msg->data[i];
//         float angle = msg->data[i + 1];
//         distances_minus150_to_150.emplace_back(distance, angle);
//     }
// }

// void fuzzy_callback(const std_msgs::Float32::ConstPtr& msg){
//     k = msg->data;
//     ROS_DEBUG("SFM received fuzzy gain = %f", k);
// }

// void vehicleSpeedCallback(const std_msgs::Float64::ConstPtr& msg) {
//     current_vehicle_speed = static_cast<float>(msg->data);
//     if (current_vehicle_speed < 0) {
//         current_vehicle_speed = 0; // Pastikan tidak negatif
//     }
// }

// void v0Callback(const std_msgs::Float64::ConstPtr& msg) {
//     // Gunakan 0.1 sebagai batas minimum untuk menghindari pembagian dengan nol
//     current_v0 = (msg->data > 0.1) ? static_cast<float>(msg->data) : 0.1f;
// }

// float calculateStaticCoordinates(float degree, float a = 1.5, float b = 0.5) {
//     float angle = degree * DEG_TO_RAD;
//     float r = (a * b) / std::sqrt(std::pow(b * std::cos(angle), 2) + std::pow(a * std::sin(angle), 2));
//     return r;
// }

// float calculateDynamicCoordinates(float degree) {
//     const float d0 = 1.5f;    // Standstill distance / jarak dasar
//     const float Th = 0.75f;   // Time headway ACC
//     const float a_min = 1.5f; // Batas minimum sumbu semi-mayor
//     const float a_max = 3.0f; // Batas maksimum sumbu semi-mayor
//     const float b0 = 0.5f;    // Sumbu semi-minor tetap

//     // Proksemik dinamis memakai prinsip constant time headway pada ACC:
//     // a_dyn = clip(d0 + Th * v_robot, a_min, a_max).
//     // v_robot berasal dari encoder, sehingga bentuk elips tidak bergantung pada target speed / v0.
//     float a_dyn = d0 + (Th * current_vehicle_speed);
//     if (a_dyn < a_min) {
//         a_dyn = a_min;
//     } else if (a_dyn > a_max) {
//         a_dyn = a_max;
//     }

//     // Rumus elips
//     float angle = degree * DEG_TO_RAD;
//     float r = (a_dyn * b0) / std::sqrt(std::pow(b0 * std::cos(angle), 2) + std::pow(a_dyn * std::sin(angle), 2));
//     return r;
// }

// float forceSocial(float distance, float degree) {
//     float psi = 0.4; 

//     float r_value = calculateStaticCoordinates(degree); //ubah ke dinamis
//     float force = k * std::exp((r_value - distance) / psi);

//     return force;
// }

// float forcePhysical(float distance, float degree) {
//     float r_value = calculateStaticCoordinates(degree);
//     float force = k * (r_value - distance);

//     return force;
// }

// void sfm() {
//     std::vector<float> distances = {0.0}; 
//     std::vector<float> angles = {180.0}; 

//     float nearest_distance_raw = 0.0; // Default large distance
//     float nearest_angle = 0.0;

//     bool isOverlap = false;

//     auto nearest_minus150_to_150 = std::min_element(
//         distances_minus150_to_150.begin(), distances_minus150_to_150.end(),
//         [](const auto& a, const auto& b) { return a.first < b.first; } 
//     );

//     if (nearest_minus150_to_150 == distances_minus150_to_150.end()) {
//         ROS_DEBUG("SFM: No obstacle data available");
//         nearest_distance_raw = 10.0;
//         nearest_angle = 180.0;

//         // Reset filter
//         history_jarak.clear();
//         sdh_diinisialisasi = false;
//     }
//     else{
//         nearest_distance_raw = nearest_minus150_to_150->first;
//         nearest_angle = nearest_minus150_to_150->second;
//     }

//     // --- (BLOK FILTER BARU) ---
//     // Terapkan Filter Moving Average
//     double jarak_mentah = static_cast<double>(nearest_distance_raw);
//     history_jarak.push_back(jarak_mentah);
//     if (history_jarak.size() > filter_size) {
//         history_jarak.pop_front();
//     }

//     double jarak_terfilter = 0.0;
//     if (!history_jarak.empty()) {
//         jarak_terfilter = std::accumulate(history_jarak.begin(), history_jarak.end(), 0.0) / history_jarak.size();
//     } else {
//         jarak_terfilter = jarak_mentah;
//     }
    

//     // --- (BLOK DELTA_DISTANCE BARU) ---
//     double jarak_sekarang_terfilter = jarak_terfilter;
//     double delta_distance = 0.0;

//     std_msgs::Float32 filtered_distance_msg;
//     filtered_distance_msg.data = static_cast<float>(jarak_sekarang_terfilter);
//     distance_filtered_publisher.publish(filtered_distance_msg);

//     // Hanya hitung delta jika filter sudah "penuh"
//     if (sdh_diinisialisasi && history_jarak.size() >= filter_size) {
//         delta_distance = (jarak_sekarang_terfilter - jarak_sebelumnya_terfilter) / dt;
//     }
    
//     jarak_sebelumnya_terfilter = jarak_sekarang_terfilter; // Selalu update

//     // Set flag inisialisasi hanya jika filter sudah penuh
//     if (history_jarak.size() >= filter_size) {
//         sdh_diinisialisasi = true;
//     } else {
//         sdh_diinisialisasi = false; // Belum siap
//     }
//     // --- (AKHIR BLOK DELTA_DISTANCE) ---

//     //LIMIT LIDAR KE Drk
//     double d_input = jarak_sekarang_terfilter;
//     const double max_change = 0.025;
    
//     if (!d_initialized){
//     	d_stable = d_input;
//     	d_initialized = true;
//     }

//     double selisih = d_input - d_stable;
//     if (selisih > max_change) {
//         selisih = max_change;
//     }
//     d_stable += selisih;

//     // Delta distance untuk fuzzy berbasis d_stable
//     if (d_delta_initialized) {
//         delta_distance = (d_stable - d_stable_sebelumnya) / dt;
//     } else {
//         delta_distance = 0.0;
//         d_delta_initialized = true;
//     }
//     d_stable_sebelumnya = d_stable;

    

//     std_msgs::Float32 stable_distance_msg;
//     stable_distance_msg.data = static_cast<float>(d_stable);
//     stable_distance_publisher.publish(stable_distance_msg);


//     // Publish nearest distance and delta_distance for fuzzy controller
//     std_msgs::Float32MultiArray dmsg;
//     dmsg.data.clear();
//     dmsg.data.push_back(static_cast<float>(d_stable));
//     dmsg.data.push_back(static_cast<float>(delta_distance));
//     near_distance.publish(dmsg);

//     float r_robot = calculateStaticCoordinates(nearest_angle);  //dynamic        
//     float r_obstacle = calculateStaticCoordinates(nearest_angle + 180); 
        
//     float effective_distance = d_stable - r_obstacle;
//     if (effective_distance < r_robot) {
//         isOverlap = true;
//     }    

//     // Calculate social force always (controlled by fuzzy gain k)
//     float f_social = forceSocial(effective_distance, nearest_angle);
    
//     // Initialize physical force
//     float f_physical = 0.0;
    
//     // Calculate physical force only when overlap occurs (emergency brake)
//     if (isOverlap == true) {
//         f_physical = forcePhysical(effective_distance, nearest_angle);
//     }
    
//     // Combine forces: social force (always) + physical force (only when overlap)
//     float f_force_static = f_social + f_physical;
//     float f_angle_reverse = nearest_angle * -1;

//     distances.push_back(f_force_static);
//     angles.push_back(f_angle_reverse);

//     std::vector<float> angle_array;
//     for (float angle : angles) {
//         angle_array.push_back(angle * DEG_TO_RAD);
//     }

//     std::vector<float> Fx, Fy;
//     for (size_t i = 0; i < distances.size(); ++i) {
//         Fx.push_back(distances[i] * cos(angle_array[i]));
//         Fy.push_back(distances[i] * sin(angle_array[i]));
//     }

//     float resultant_vector_x = std::accumulate(Fx.begin(), Fx.end(), 0.0);
//     float resultant_vector_y = std::accumulate(Fy.begin(), Fy.end(), 0.0);
//     resultant_magnitude = sqrt(resultant_vector_x * resultant_vector_x + resultant_vector_y * resultant_vector_y);
//     resultant_angle = atan2(resultant_vector_y, resultant_vector_x);
//     resultant_angle = resultant_angle * RAD_TO_DEG;

//     // Publish resultant vector components
//     std_msgs::Float32 xmsg;
//     xmsg.data = resultant_vector_x;
    
//     vector_x_publisher.publish(xmsg);

//     std_msgs::Float32 ymsg;
//     ymsg.data = resultant_vector_y;
//     vector_y_publisher.publish(ymsg);

//     // Publish magnitude for drive controller
//     geometry_msgs::Point magnitude_msg;
//     magnitude_msg.x = resultant_magnitude;
//     magnitude_msg.y = 0;
//     magnitude_msg.z = 0;
//     magnitude_publisher.publish(magnitude_msg);

//     // Publish direction for drive controller
//     geometry_msgs::Point direction_msg;
//     direction_msg.x = resultant_angle;
//     direction_msg.y = 0;
//     direction_msg.z = 0;
//     direction_publisher.publish(direction_msg);
// }

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "sfm_node");
//     ros::NodeHandle n;

//     // Publishers - Output to Drive and Fuzzy controllers
//     magnitude_publisher = n.advertise<geometry_msgs::Point>("/fnav", 1000);
//     direction_publisher = n.advertise<geometry_msgs::Point>("/rho", 1000);
//     vector_y_publisher = n.advertise<std_msgs::Float32>("/resultant_vector_y", 1000);
//     vector_x_publisher = n.advertise<std_msgs::Float32>("/resultant_vector_x", 1000);
//     near_distance = n.advertise<std_msgs::Float32MultiArray>("/near_distance", 1000);
//     stable_distance_publisher = n.advertise<std_msgs::Float32>("/d_stable", 1000);
//     distance_filtered_publisher = n.advertise<std_msgs::Float32>("/distance_filtered", 1000);

//     // Subscribers - Input from Lidar Scan and Fuzzy controllers
//     ros::Subscriber sub_distances_minus150_to_150 = n.subscribe("/distances_minus150_to_150", 1000, distancesMinus150to150Callback);
//     ros::Subscriber sub_fuzzy = n.subscribe("/fuzzy", 10, fuzzy_callback);
//     ros::Subscriber sub_speed = n.subscribe("/vehicle_speed", 10, vehicleSpeedCallback);
//     ros::Subscriber sub_v0 = n.subscribe("/v0_target", 10, v0Callback);

//     ros::Rate loop_rate(30);
//     while (ros::ok()) {
//         sfm();
//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     return 0;
// }
