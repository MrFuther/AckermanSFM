#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include <iostream>
#include <vector>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <utility>

ros::Publisher magnitude_publisher;
ros::Publisher direction_publisher;
ros::Publisher vector_y_publisher;
ros::Publisher vector_x_publisher;
ros::Publisher near_distance;
ros::Publisher active_object_echo_publisher; 
ros::Publisher f_obs_y_publisher;
ros::Publisher f_lane_y_publisher;
ros::Publisher force_components_publisher;
ros::Subscriber sub_lane_error;

std::vector<std::pair<float, float>> distances_minus150_to_150;

float previous_distance = 10.0f;
float current_distance = 10.0f;
float resultant_magnitude = 0.0;
float resultant_angle = 0.0;
float k = 0.0; 
float current_lane_error = 0.0; 
float current_v0 = 1.0f; 

int current_active_object = -1; 

struct ObjectParameters {
    float semi_axis_a;    
    float semi_axis_b;    
    float psi;            
    float k_multiplier;   
    std::string name;
};

std::vector<ObjectParameters> object_params = {
    {1.2f, 1.0f, 0.30f, 1.5f, "Bus"},        
    {1.0f, 0.5f, 0.25f, 1.0f, "Car"},        
    {0.9f, 0.5f, 0.15f, 1.0f, "Motorcycle"}, 
    {0.9f, 0.5f, 0.15f, 1.0f, "Person"},     
    {1.2f, 1.0f, 0.30f, 2.0f, "Truck"}       
};

ObjectParameters default_params = {1.0f, 0.5f, 0.25f, 1.0f, "Default"};

void distancesMinus150to150Callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    distances_minus150_to_150.clear();
    if (msg->data.size() % 2 != 0) return;
    for (size_t i = 0; i < msg->data.size(); i += 2) {
        distances_minus150_to_150.emplace_back(msg->data[i], msg->data[i + 1]);
    }
}

void laneErrorCallback(const std_msgs::Float32::ConstPtr& msg) {
    current_lane_error = msg->data;
}

void fuzzy_callback(const std_msgs::Float32::ConstPtr& msg){
    k = msg->data;
}

void activeObjectCallback(const std_msgs::Int32::ConstPtr& msg) {
    current_active_object = msg->data;
}

void v0Callback(const std_msgs::Float64::ConstPtr& msg) {
    current_v0 = msg->data;
}

ObjectParameters getActiveParameters() {
    if (current_active_object >= 0 && current_active_object < object_params.size()) {
        return object_params[current_active_object];
    }
    return default_params;
}

float calculateCoordinates(float degree, float a, float b) {
    float angle = degree * M_PI / 180.0;
    return (a * b) / std::sqrt(std::pow(b * std::cos(angle), 2) + std::pow(a * std::sin(angle), 2));
}

float forceSocial(float distance, float degree, float psi, float k_effective) {
    ObjectParameters params = getActiveParameters();
    float r_value = calculateCoordinates(degree, params.semi_axis_a, params.semi_axis_b);
    return k_effective * std::exp((r_value - distance) / psi);
}

float forcePhysical(float distance, float degree, float k_effective) {
    ObjectParameters params = getActiveParameters();
    float r_value = calculateCoordinates(degree, params.semi_axis_a, params.semi_axis_b);
    return k_effective * (r_value - distance);
}

void sfm() {
    float nearest_distance = 10.0;
    float nearest_angle = 180.0;
    bool isOverlap = false;
    ObjectParameters params = getActiveParameters();

    auto nearest_minus150_to_150 = std::min_element(
        distances_minus150_to_150.begin(), distances_minus150_to_150.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; } 
    );

    if (nearest_minus150_to_150 != distances_minus150_to_150.end()){
        nearest_distance = nearest_minus150_to_150->first;
        nearest_angle = nearest_minus150_to_150->second;
    }

    previous_distance = current_distance;
    current_distance = nearest_distance;
    float delta_distance = current_distance - previous_distance;

    std_msgs::Float32MultiArray dmsg;
    dmsg.data.push_back(nearest_distance);
    dmsg.data.push_back(delta_distance);
    near_distance.publish(dmsg);

    // Kalkulasi Proxemics Robot Dinamis berdasarkan v0
    float v0_clamped = std::max(0.3f, std::min(1.0f, current_v0));
    float robot_a = 0.5f + (v0_clamped - 0.3f) * (1.0f - 0.5f) / (1.0f - 0.3f);
    float robot_b = 0.3f + (v0_clamped - 0.3f) * (0.5f - 0.3f) / (1.0f - 0.3f);

    float r_robot = calculateCoordinates(nearest_angle, robot_a, robot_b);          
    float r_obstacle = calculateCoordinates(nearest_angle + 180, params.semi_axis_a, params.semi_axis_b); 
    float effective_distance = nearest_distance - r_obstacle;
    if (effective_distance < r_robot) isOverlap = true;

    // =============================================================
    // 1. KALKULASI GAYA TOLAK MENTAH (MAGNITUDO)
    // =============================================================
    float k_effective = k * params.k_multiplier;
    float f_force_static = 0.0;

    if (isOverlap == true) {
        float f_social = forceSocial(effective_distance, nearest_angle, params.psi, k_effective);
        float f_physical = forcePhysical(effective_distance, nearest_angle, k_effective);
        f_force_static = f_social + f_physical;
    }

    // =============================================================
    // 2. COUPLED SFM: SUPERPOSISI VEKTOR (X & Y)
    // =============================================================
    // Konstanta Vektor
    float F_GOAL_X = 50.0;     // Dorongan mesin selalu ke depan (Sumbu X positif)
    float MAX_F_LANE = 40.0;   // Tarikan garis maksimal (Sumbu Y)

    // A. Dekomposisi Gaya Objek (F_obs)
    // nearest_angle: 180 adalah lurus depan. <180 Kiri, >180 Kanan.
    float obs_theta_rad = (nearest_angle - 180.0) * M_PI / 180.0;
    
    // Gaya tolak rintangan mendorong *berlawanan* arah dengan posisi rintangan
    float F_obs_x = -f_force_static * cos(obs_theta_rad); 
    float F_obs_y = -f_force_static * sin(obs_theta_rad); 

    // B. Dekomposisi Gaya Garis (F_lane)
    float F_lane_y = current_lane_error * MAX_F_LANE;

    // C. Penggabungan Vektor Total
    float F_total_x = F_GOAL_X + F_obs_x;
    float F_total_y = F_lane_y + F_obs_y;

    // Proteksi: Cegah F_total_x negatif agar sudut atan2 tidak berbalik arah 
    // (Mencegah setir bergerak aneh saat robot ngerem total)
    if (F_total_x < 0.1) F_total_x = 0.1;

    // =============================================================
    // 3. PENENTUAN OUTPUT UNTUK DRIVE.CPP
    // =============================================================
    // /fnav di drive.cpp bertindak sebagai "Pinalti Rem" yang mengurangi v0.
    // Pinalti X ini adalah selisih antara Gaya Target (50) dengan Gaya Total X.
    resultant_magnitude = F_GOAL_X - F_total_x; 

    // /rho dikalkulasi murni dari rasio Trigonometri vektor Y terhadap vektor X
    float angle_offset = atan2(F_total_y, F_total_x) * (180.0 / M_PI);
    resultant_angle = 180.0 + angle_offset;

    // Publish data
    std_msgs::Float32 xmsg, ymsg;
    xmsg.data = resultant_magnitude * cos(resultant_angle * M_PI / 180.0);
    ymsg.data = resultant_magnitude * sin(resultant_angle * M_PI / 180.0);
    vector_x_publisher.publish(xmsg);
    vector_y_publisher.publish(ymsg);

    geometry_msgs::Point magnitude_msg;
    magnitude_msg.x = resultant_magnitude;
    magnitude_publisher.publish(magnitude_msg);

    geometry_msgs::Point direction_msg;
    direction_msg.x = resultant_angle; 
    direction_publisher.publish(direction_msg);

    std_msgs::Int32 echo_msg;
    echo_msg.data = current_active_object;
    active_object_echo_publisher.publish(echo_msg);

    std_msgs::Float32 f_obs_y_msg;
    f_obs_y_msg.data = F_obs_y;
    f_obs_y_publisher.publish(f_obs_y_msg);

    std_msgs::Float32 f_lane_y_msg;
    f_lane_y_msg.data = F_lane_y;
    f_lane_y_publisher.publish(f_lane_y_msg);

    // Publish force components for animationnew.py
    std_msgs::Float32MultiArray force_msg;
    force_msg.data.push_back(F_GOAL_X);
    force_msg.data.push_back(F_lane_y);
    force_msg.data.push_back(F_obs_x);
    force_msg.data.push_back(F_obs_y);
    force_components_publisher.publish(force_msg);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sfm_node");
    ros::NodeHandle n;

    magnitude_publisher = n.advertise<geometry_msgs::Point>("/fnav", 1000);
    direction_publisher = n.advertise<geometry_msgs::Point>("/rho", 1000);
    vector_y_publisher = n.advertise<std_msgs::Float32>("/resultant_vector_y", 1000);
    vector_x_publisher = n.advertise<std_msgs::Float32>("/resultant_vector_x", 1000);
    near_distance = n.advertise<std_msgs::Float32MultiArray>("/near_distance", 1000);
    active_object_echo_publisher = n.advertise<std_msgs::Int32>("/active_object_type", 1000);
    f_obs_y_publisher = n.advertise<std_msgs::Float32>("/sfm/f_obs_y", 1000);
    f_lane_y_publisher = n.advertise<std_msgs::Float32>("/sfm/f_lane_y", 1000);
    force_components_publisher = n.advertise<std_msgs::Float32MultiArray>("/force_components", 1000);

    ros::Subscriber sub_distances_minus150_to_150 = n.subscribe("/distances_minus150_to_150", 1000, distancesMinus150to150Callback);
    ros::Subscriber sub_fuzzy = n.subscribe("/fuzzy", 10, fuzzy_callback);
    ros::Subscriber sub_active_object = n.subscribe("/active_object_type", 10, activeObjectCallback);
    ros::Subscriber sub_lane_error = n.subscribe("/f_lane_fuzzy", 10, laneErrorCallback);
    ros::Subscriber sub_v0 = n.subscribe("/v0_target", 10, v0Callback);

    ros::Rate loop_rate(30);
    while (ros::ok()) {
        sfm();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}