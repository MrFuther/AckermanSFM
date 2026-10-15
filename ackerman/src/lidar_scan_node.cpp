#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64.h"
#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <numeric>
#include <utility>
#include <vector>

ros::Publisher pub_all_distances;
ros::Publisher pub_all_angles;
ros::Publisher pub_distances_90_to_180;
ros::Publisher pub_distances_minus180_to_minus90;
ros::Publisher pub_distances_minus150_to_150;
ros::Publisher pub_front_obstacle_distance;
ros::Publisher pub_obstacle_relative_speed;
ros::Publisher pub_obstacle_speed_from_lidar;
ros::Publisher pub_front_obstacle_distance_trimmed;
ros::Publisher pub_d_dot_slope;
ros::Publisher pub_obstacle_speed_raw;
ros::Publisher pub_obstacle_speed_used;

float current_vehicle_speed = 0.0f;
static std::deque<std::pair<ros::Time, float>> front_distance_samples;
static float previous_d_dot_estimated = 0.0f;
static float previous_obstacle_speed_filtered = 0.0f;
static float previous_obstacle_speed_used = 0.0f;
static bool obstacle_speed_filter_initialized = false;
static bool obstacle_speed_used_initialized = false;

const float MAX_DETECTION_RANGE = 5.0f;
const float FRONT_CLUSTER_THRESHOLD = 0.15f;
const size_t SLOPE_WINDOW_SIZE = 5;
const float OBSTACLE_SPEED_MIN = 0.0f;
const float OBSTACLE_SPEED_MAX = 2.0f;
const float OBSTACLE_SPEED_ALPHA = 0.15f;
const float V_OBS_MAX_DELTA = 0.10f;

void vehicleSpeedCallback(const std_msgs::Float64::ConstPtr& msg) {
    current_vehicle_speed = static_cast<float>(msg->data);
    if (current_vehicle_speed < 0.0f) {
        current_vehicle_speed = 0.0f;
    }
}

bool isFrontObstacleSector(float angle_degrees) {
    return (angle_degrees >= 170.0f && angle_degrees <= 180.0f) ||
           (angle_degrees <= -170.0f && angle_degrees >= -180.0f);
}

float clampFloat(float value, float min_value, float max_value) {
    return std::max(min_value, std::min(max_value, value));
}

float calculateTrimmedMean(std::vector<float> distances) {
    if (distances.empty()) {
        return std::numeric_limits<float>::infinity();
    }

    std::sort(distances.begin(), distances.end());

    size_t start_index = 0;
    size_t end_index = distances.size();

    if (distances.size() >= 5) {
        size_t trim_count = std::max<size_t>(1, distances.size() / 10);
        if (distances.size() > 2 * trim_count) {
            start_index = trim_count;
            end_index = distances.size() - trim_count;
        }
    }

    float sum = std::accumulate(distances.begin() + start_index,
                                distances.begin() + end_index,
                                0.0f);
    return sum / static_cast<float>(end_index - start_index);
}

void resetObstacleTracking() {
    front_distance_samples.clear();
    previous_d_dot_estimated = 0.0f;
    obstacle_speed_filter_initialized = false;
    obstacle_speed_used_initialized = false;
}

float calculateDistanceSlope(const ros::Time& scan_time, float front_distance_trimmed) {
    front_distance_samples.push_back(std::make_pair(scan_time, front_distance_trimmed));
    while (front_distance_samples.size() > SLOPE_WINDOW_SIZE) {
        front_distance_samples.pop_front();
    }

    if (front_distance_samples.size() < SLOPE_WINDOW_SIZE) {
        previous_d_dot_estimated = 0.0f;
        return previous_d_dot_estimated;
    }

    double t0 = front_distance_samples.front().first.toSec();
    double t_sum = 0.0;
    double d_sum = 0.0;

    for (const auto& sample : front_distance_samples) {
        t_sum += sample.first.toSec() - t0;
        d_sum += sample.second;
    }

    double sample_count = static_cast<double>(front_distance_samples.size());
    double t_mean = t_sum / sample_count;
    double d_mean = d_sum / sample_count;
    double numerator = 0.0;
    double denominator = 0.0;

    for (const auto& sample : front_distance_samples) {
        double t = sample.first.toSec() - t0;
        double d = sample.second;
        numerator += (t - t_mean) * (d - d_mean);
        denominator += (t - t_mean) * (t - t_mean);
    }

    if (denominator <= 1e-9) {
        return previous_d_dot_estimated;
    }

    previous_d_dot_estimated = static_cast<float>(numerator / denominator);
    return previous_d_dot_estimated;
}

void publishObstacleSpeed(const sensor_msgs::LaserScan::ConstPtr& msg,
                          bool front_obstacle_found,
                          float front_distance_trimmed) {
    if (!front_obstacle_found) {
        resetObstacleTracking();

        std_msgs::Float32 distance_msg;
        distance_msg.data = MAX_DETECTION_RANGE;
        pub_front_obstacle_distance.publish(distance_msg);
        pub_front_obstacle_distance_trimmed.publish(distance_msg);

        std_msgs::Float32 d_dot_msg;
        d_dot_msg.data = 0.0f;
        pub_obstacle_relative_speed.publish(d_dot_msg);
        pub_d_dot_slope.publish(d_dot_msg);

        float obstacle_speed_raw = current_vehicle_speed;
        float obstacle_speed_used = clampFloat(obstacle_speed_raw,
                                               OBSTACLE_SPEED_MIN,
                                               OBSTACLE_SPEED_MAX);

        std_msgs::Float32 obstacle_speed_raw_msg;
        obstacle_speed_raw_msg.data = obstacle_speed_raw;
        pub_obstacle_speed_raw.publish(obstacle_speed_raw_msg);

        std_msgs::Float32 obstacle_speed_msg;
        obstacle_speed_msg.data = obstacle_speed_used;
        pub_obstacle_speed_from_lidar.publish(obstacle_speed_msg);
        pub_obstacle_speed_used.publish(obstacle_speed_msg);
        return;
    }

    ros::Time scan_time = msg->header.stamp;
    if (scan_time.isZero()) {
        scan_time = ros::Time::now();
    }

    // LiDAR sector -> d_min -> d_cluster -> d_trimmed -> linear regression slope d_dot
    // -> v_obs_raw = v_robot + d_dot -> clamp -> low-pass filter -> rate limiter
    // -> v_obs_used -> publish.
    float d_dot_estimated = calculateDistanceSlope(scan_time, front_distance_trimmed);

    std_msgs::Float32 distance_msg;
    distance_msg.data = front_distance_trimmed;
    pub_front_obstacle_distance.publish(distance_msg);
    pub_front_obstacle_distance_trimmed.publish(distance_msg);

    std_msgs::Float32 d_dot_msg;
    d_dot_msg.data = d_dot_estimated;
    pub_obstacle_relative_speed.publish(d_dot_msg);
    pub_d_dot_slope.publish(d_dot_msg);

    float obstacle_speed_raw = current_vehicle_speed + d_dot_estimated;
    float obstacle_speed_clamped = clampFloat(obstacle_speed_raw,
                                             OBSTACLE_SPEED_MIN,
                                             OBSTACLE_SPEED_MAX);

    float obstacle_speed_filtered = obstacle_speed_clamped;
    if (obstacle_speed_filter_initialized) {
        obstacle_speed_filtered = OBSTACLE_SPEED_ALPHA * obstacle_speed_clamped +
                                  (1.0f - OBSTACLE_SPEED_ALPHA) * previous_obstacle_speed_filtered;
    } else {
        obstacle_speed_filter_initialized = true;
    }
    previous_obstacle_speed_filtered = obstacle_speed_filtered;

    float obstacle_speed_used = obstacle_speed_filtered;
    if (obstacle_speed_used_initialized) {
        float delta = obstacle_speed_filtered - previous_obstacle_speed_used;
        delta = clampFloat(delta, -V_OBS_MAX_DELTA, V_OBS_MAX_DELTA);
        obstacle_speed_used = previous_obstacle_speed_used + delta;
    } else {
        obstacle_speed_used_initialized = true;
    }
    previous_obstacle_speed_used = obstacle_speed_used;

    std_msgs::Float32 obstacle_speed_raw_msg;
    obstacle_speed_raw_msg.data = obstacle_speed_raw;
    pub_obstacle_speed_raw.publish(obstacle_speed_raw_msg);

    std_msgs::Float32 obstacle_speed_msg;
    obstacle_speed_msg.data = obstacle_speed_used;
    pub_obstacle_speed_from_lidar.publish(obstacle_speed_msg);
    pub_obstacle_speed_used.publish(obstacle_speed_msg);
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    static std_msgs::Float32MultiArray all_distances_msg;
    static std_msgs::Float32MultiArray all_angles_msg;
    static std_msgs::Float32MultiArray distances_90_to_180_msg;
    static std_msgs::Float32MultiArray distances_minus180_to_minus90_msg;
    static std_msgs::Float32MultiArray distances_minus150_to_150_msg;
    static std::vector<float> front_sector_distances;

    all_distances_msg.data.clear();
    all_angles_msg.data.clear();
    distances_90_to_180_msg.data.clear();
    distances_minus180_to_minus90_msg.data.clear();
    distances_minus150_to_150_msg.data.clear();
    front_sector_distances.clear();

    const std::vector<float>& ranges = msg->ranges;
    if (all_distances_msg.data.capacity() < ranges.size()) {
        all_distances_msg.data.reserve(ranges.size());
        all_angles_msg.data.reserve(ranges.size());
        distances_90_to_180_msg.data.reserve(ranges.size());
        distances_minus180_to_minus90_msg.data.reserve(ranges.size());
        distances_minus150_to_150_msg.data.reserve(ranges.size());
        front_sector_distances.reserve(ranges.size());
    }

    float angle_increment = msg->angle_increment;
    float angle_min = msg->angle_min;
    float front_distance_trimmed = std::numeric_limits<float>::infinity();
    bool front_obstacle_found = false;

    for (size_t i = 0; i < ranges.size(); ++i) {
        float distance = ranges[i];
        if (!std::isfinite(distance) ||
            distance < msg->range_min ||
            distance > msg->range_max ||
            distance > MAX_DETECTION_RANGE) {
            continue;
        }

        float angle = angle_min + i * angle_increment;
        float angle_degrees = angle * 180.0 / M_PI;

        all_distances_msg.data.push_back(distance);
        all_angles_msg.data.push_back(angle_degrees);

        if (angle_degrees >= 90 && angle_degrees <= 150) {
            distances_90_to_180_msg.data.push_back(distance);
            distances_90_to_180_msg.data.push_back(angle_degrees);
        } else if (angle_degrees >= -150 && angle_degrees <= -90) {
            distances_minus180_to_minus90_msg.data.push_back(distance);
            distances_minus180_to_minus90_msg.data.push_back(angle_degrees);
        }

        if (isFrontObstacleSector(angle_degrees)) {
            distances_minus150_to_150_msg.data.push_back(distance);
            distances_minus150_to_150_msg.data.push_back(angle_degrees);
            front_sector_distances.push_back(distance);
        }
    }

    if (!front_sector_distances.empty()) {
        float d_min = *std::min_element(front_sector_distances.begin(), front_sector_distances.end());
        std::vector<float> front_cluster_distances;
        front_cluster_distances.reserve(front_sector_distances.size());

        for (float distance : front_sector_distances) {
            if (distance <= d_min + FRONT_CLUSTER_THRESHOLD) {
                front_cluster_distances.push_back(distance);
            }
        }

        if (!front_cluster_distances.empty()) {
            front_distance_trimmed = calculateTrimmedMean(front_cluster_distances);
            front_obstacle_found = true;
        }
    }

    pub_all_distances.publish(all_distances_msg);

    pub_all_angles.publish(all_angles_msg);

    pub_distances_90_to_180.publish(distances_90_to_180_msg);

    pub_distances_minus180_to_minus90.publish(distances_minus180_to_minus90_msg);

    pub_distances_minus150_to_150.publish(distances_minus150_to_150_msg);

    publishObstacleSpeed(msg, front_obstacle_found, front_distance_trimmed);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_scan_node");
    ros::NodeHandle n;

    pub_all_distances = n.advertise<std_msgs::Float32MultiArray>("/all_distances", 10);
    pub_all_angles = n.advertise<std_msgs::Float32MultiArray>("/all_angles", 10);
    pub_distances_90_to_180 = n.advertise<std_msgs::Float32MultiArray>("/distances_90_to_180", 10);
    pub_distances_minus180_to_minus90 = n.advertise<std_msgs::Float32MultiArray>("/distances_minus180_to_minus90", 10);
    pub_distances_minus150_to_150 = n.advertise<std_msgs::Float32MultiArray>("/distances_minus150_to_150", 10);
    pub_front_obstacle_distance = n.advertise<std_msgs::Float32>("/front_obstacle_distance", 10);
    pub_obstacle_relative_speed = n.advertise<std_msgs::Float32>("/obstacle_relative_speed", 10);
    pub_obstacle_speed_from_lidar = n.advertise<std_msgs::Float32>("/obstacle_speed_from_lidar", 10);
    pub_front_obstacle_distance_trimmed = n.advertise<std_msgs::Float32>("/front_obstacle_distance_trimmed", 10);
    pub_d_dot_slope = n.advertise<std_msgs::Float32>("/d_dot_slope", 10);
    pub_obstacle_speed_raw = n.advertise<std_msgs::Float32>("/obstacle_speed_raw", 10);
    pub_obstacle_speed_used = n.advertise<std_msgs::Float32>("/obstacle_speed_used", 10);

    ros::Subscriber sub = n.subscribe("/scan", 10, laserCallback);
    ros::Subscriber sub_speed = n.subscribe("/vehicle_speed", 10, vehicleSpeedCallback);
    ros::spin();

    return 0;
}

// #include "ros/ros.h"
// #include "sensor_msgs/LaserScan.h"
// #include "std_msgs/Float32MultiArray.h"
// #include "geometry_msgs/Point.h"
// #include <cmath>
// #include <iostream>
// #include <vector>
// #include <utility>

// ros::Publisher pub_all_distances;
// ros::Publisher pub_all_angles;
// ros::Publisher pub_distances_90_to_180;
// ros::Publisher pub_distances_minus180_to_minus90;
// ros::Publisher pub_distances_minus150_to_150;

// void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
//     std::vector<float> all_distances;
//     std::vector<float> all_angles;
//     std::vector<std::pair<float, float>> distances_90_to_180;
//     std::vector<std::pair<float, float>> distances_minus180_to_minus90;
//     std::vector<std::pair<float, float>> distances_minus150_to_150;

//     distances_90_to_180.clear();
//     distances_minus180_to_minus90.clear();
//     distances_minus150_to_150.clear();
//     all_distances.clear();
//     all_angles.clear();

//     const std::vector<float>& ranges = msg->ranges;
//     float angle_increment = msg->angle_increment;
//     float angle_min = msg->angle_min;

//     for (size_t i = 0; i < ranges.size(); ++i) {
//         float distance = ranges[i];
//         if (distance > 5.0) continue;

//         float angle = angle_min + i * angle_increment;
//         float angle_degrees = angle * 180.0 / M_PI; // Convert to degrees

//         // if (angle_degrees <= 0) {
//         //     angle_degrees += 180;
//         // } 
//         // else if (angle_degrees >= 0) {
//         //     angle_degrees -= 180;
//         // }

//         all_distances.push_back(distance);
//         all_angles.push_back(angle_degrees);

//         if (angle_degrees >= 90 && angle_degrees <= 150) {
//             distances_90_to_180.emplace_back(distance, angle_degrees);
//         } 
//         else if (angle_degrees >= -150 && angle_degrees <= -90) {
//             distances_minus180_to_minus90.emplace_back(distance, angle_degrees);
//         }
//         // if (angle_degrees >= -150 && angle_degrees <= 150) {
//         //     distances_minus150_to_150.emplace_back(distance, angle_degrees);
//         // } 
//         if (angle_degrees <= -170 && angle_degrees >= -180 || angle_degrees <= 180 && angle_degrees >= 170) {
//             distances_minus150_to_150.emplace_back(distance, angle_degrees);
//         }
//     }

//     std_msgs::Float32MultiArray all_distances_msg;
//     all_distances_msg.data = all_distances;
//     pub_all_distances.publish(all_distances_msg);

//     std_msgs::Float32MultiArray all_angles_msg;
//     all_angles_msg.data = all_angles;
//     pub_all_angles.publish(all_angles_msg);

//     std_msgs::Float32MultiArray distances_90_to_180_msg;
//     for (const auto& pair : distances_90_to_180) {
//         distances_90_to_180_msg.data.push_back(pair.first);
//         distances_90_to_180_msg.data.push_back(pair.second);
//     }
//     pub_distances_90_to_180.publish(distances_90_to_180_msg);

//     std_msgs::Float32MultiArray distances_minus180_to_minus90_msg;
//     for (const auto& pair : distances_minus180_to_minus90) {
//         distances_minus180_to_minus90_msg.data.push_back(pair.first);
//         distances_minus180_to_minus90_msg.data.push_back(pair.second);
//     }
//     pub_distances_minus180_to_minus90.publish(distances_minus180_to_minus90_msg);

//     // for (const auto& distance : distances_minus150_to_150) {
//     //     std::cout << "Distance: " << distance.first
//     //               << ", Angle: " << distance.second << '\n';
//     // }

//     std_msgs::Float32MultiArray distances_minus150_to_150_msg;
//     for (const auto& pair : distances_minus150_to_150) {
//         distances_minus150_to_150_msg.data.push_back(pair.first);
//         distances_minus150_to_150_msg.data.push_back(pair.second);
//     }
//     pub_distances_minus150_to_150.publish(distances_minus150_to_150_msg);
// }

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "lidar_scan_node");
//     ros::NodeHandle n;

//     pub_all_distances = n.advertise<std_msgs::Float32MultiArray>("/all_distances", 1000);
//     pub_all_angles = n.advertise<std_msgs::Float32MultiArray>("/all_angles", 1000);
//     pub_distances_90_to_180 = n.advertise<std_msgs::Float32MultiArray>("/distances_90_to_180", 1000);
//     pub_distances_minus180_to_minus90 = n.advertise<std_msgs::Float32MultiArray>("/distances_minus180_to_minus90", 1000);
//     pub_distances_minus150_to_150 = n.advertise<std_msgs::Float32MultiArray>("/distances_minus150_to_150", 1000); // New publisher

//     ros::Subscriber sub = n.subscribe("/scan", 1000, laserCallback);
//     ros::spin();

//     return 0;
// }