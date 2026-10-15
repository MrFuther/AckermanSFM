#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32.h"
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

std::vector<std::pair<float, float>> distances_minus150_to_150;

float resultant_magnitude = 0.0;
float resultant_angle = 0.0;
float k = 0.0; // Default fuzzy gain

void distancesMinus150to150Callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    distances_minus150_to_150.clear();

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

float calculateCoordinates(float degree, float a = 1, float b = 0.5) {
    float angle = degree * M_PI / 180.0;
    float r = (a * b) / std::sqrt(std::pow(b * std::cos(angle), 2) + std::pow(a * std::sin(angle), 2));
    return r;
}

float forceSocial(float distance, float degree) {
    float psi = 0.20;

    float r_value = calculateCoordinates(degree);
    float force = k * std::exp((r_value - distance) / psi);

    return force;
}

float forcePhysical(float distance, float degree) {
    float r_value = calculateCoordinates(degree);
    float force = k * (r_value - distance);

    return force;
}

void sfm() {
    std::vector<float> distances = {0.0}; 
    std::vector<float> angles = {180.0}; 

    float nearest_distance = 0.0; // Default large distance
    float nearest_angle = 0.0;

    bool isOverlap = false;

    auto nearest_minus150_to_150 = std::min_element(
        distances_minus150_to_150.begin(), distances_minus150_to_150.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; } 
    );

    if (nearest_minus150_to_150 == distances_minus150_to_150.end()) {
        ROS_DEBUG("SFM: No obstacle data available");
        nearest_distance = 10.0;
        nearest_angle = 180.0;
    }
    else{
        nearest_distance = nearest_minus150_to_150->first;
        nearest_angle = nearest_minus150_to_150->second;
    }

    // Publish nearest distance for fuzzy controller
    std_msgs::Float32 dmsg;
    dmsg.data = nearest_distance;
    near_distance.publish(dmsg);

    float r_robot = calculateCoordinates(nearest_angle);          
    float r_obstacle = calculateCoordinates(nearest_angle + 180); 
        
    float effective_distance = nearest_distance - r_obstacle;
    if (effective_distance < r_robot) {
        isOverlap = true;
    }    

    if (isOverlap == true) {
        float f_social = forceSocial(effective_distance, nearest_angle);
        float f_physical = forcePhysical(effective_distance, nearest_angle);
        float f_force_static = f_social + f_physical;
        float f_angle_reverse = nearest_angle * -1;

        distances.push_back(f_force_static);
        angles.push_back(f_angle_reverse);
    }

    std::vector<float> angle_array;
    for (float angle : angles) {
        angle_array.push_back(angle * M_PI / 180.0);
    }

    std::vector<float> Fx, Fy;
    for (size_t i = 0; i < distances.size(); ++i) {
        Fx.push_back(distances[i] * cos(angle_array[i]));
        Fy.push_back(distances[i] * sin(angle_array[i]));
    }

    float resultant_vector_x = std::accumulate(Fx.begin(), Fx.end(), 0.0);
    float resultant_vector_y = std::accumulate(Fy.begin(), Fy.end(), 0.0);
    resultant_magnitude = sqrt(resultant_vector_x * resultant_vector_x + resultant_vector_y * resultant_vector_y);
    resultant_angle = atan2(resultant_vector_y, resultant_vector_x);
    resultant_angle = resultant_angle * 180.0 / M_PI;

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

    // Publishers - Output to Drive and Fuzzy controllers
    magnitude_publisher = n.advertise<geometry_msgs::Point>("/fnav", 1000);
    direction_publisher = n.advertise<geometry_msgs::Point>("/rho", 1000);
    vector_y_publisher = n.advertise<std_msgs::Float32>("/resultant_vector_y", 1000);
    vector_x_publisher = n.advertise<std_msgs::Float32>("/resultant_vector_x", 1000);
    near_distance = n.advertise<std_msgs::Float32>("/near_distance", 1000);

    // Subscribers - Input from Lidar Scan and Fuzzy controllers
    ros::Subscriber sub_distances_minus150_to_150 = n.subscribe("/distances_minus150_to_150", 1000, distancesMinus150to150Callback);
    ros::Subscriber sub_fuzzy = n.subscribe("/fuzzy", 10, fuzzy_callback);

    ros::Rate loop_rate(30);
    while (ros::ok()) {
        sfm();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}