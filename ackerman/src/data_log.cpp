#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float32MultiArray.h>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <mutex>
#include <limits>
#include <cmath>

// Define your logging mode (uncomment one)
#define LOG_FNAV_MODE
// #define LOG_ENCODER_MODE

#ifdef LOG_FNAV_MODE
#include <geometry_msgs/Point.h>
#endif

std::ofstream log_file;
std::string filename;
std::mutex log_mutex;
bool command_received = false;
double latest_obstacle_angle_deg = 0.0;
bool obstacle_angle_valid = false;

#ifdef LOG_FNAV_MODE
// FNAV mode variables
std_msgs::Float64::ConstPtr latest_velocity_cmd;
std_msgs::Float32MultiArray::ConstPtr latest_distance;
std_msgs::Float32::ConstPtr latest_distance_filtered;
std_msgs::Float32::ConstPtr latest_d_stable;
std_msgs::Float32::ConstPtr fuzzy_gain;
geometry_msgs::Point::ConstPtr latest_fnav;
std_msgs::Float64::ConstPtr latest_real_velocity;
std_msgs::Float32::ConstPtr latest_a_dyn;
std_msgs::Float32::ConstPtr latest_obstacle_speed_from_lidar;
#endif

#ifdef LOG_ENCODER_MODE
// Encoder mode variables
std_msgs::Float64::ConstPtr latest_velopub;
std_msgs::UInt16::ConstPtr latest_v1;
std_msgs::Float64::ConstPtr latest_speed;
#endif

std::string getROSTimestamp() {
    ros::Time now = ros::Time::now();
    std::time_t seconds = now.sec;
    std::tm* time_info = std::localtime(&seconds);
    std::ostringstream timestamp;
    timestamp << std::put_time(time_info, "%Y-%m-%d %H:%M:%S");
    timestamp << "." << std::setw(9) << std::setfill('0') << now.nsec;
    return timestamp.str();
}

#ifdef LOG_FNAV_MODE
// FNAV mode callbacks
void fnavCallback(const geometry_msgs::Point::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    latest_fnav = msg;
}

void distanceCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    latest_distance = msg;
}

void filteredDistanceCallback(const std_msgs::Float32::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    latest_distance_filtered = msg;
}

void stableDistanceCallback(const std_msgs::Float32::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    latest_d_stable = msg;
}

void velocityCmdCallback(const std_msgs::Float64::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    latest_velocity_cmd = msg;
}

void realVelocityCallback(const std_msgs::Float64::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    latest_real_velocity = msg;
}

void aDynCallback(const std_msgs::Float32::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    latest_a_dyn = msg;
}

void obstacleSpeedFromLidarCallback(const std_msgs::Float32::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    latest_obstacle_speed_from_lidar = msg;
}

void fuzzy_callback(const std_msgs::Float32::ConstPtr& msg){
    std::lock_guard<std::mutex> lock(log_mutex);
    fuzzy_gain = msg;
}

void obstacleAngleCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);

    if (msg->data.size() < 2) {
        obstacle_angle_valid = false;
        return;
    }

    float nearest_distance = std::numeric_limits<float>::infinity();
    float nearest_angle = 0.0f;
    bool found_valid_pair = false;

    // Expected format: [distance_0, angle_0, distance_1, angle_1, ...]
    for (size_t i = 0; i + 1 < msg->data.size(); i += 2) {
        const float distance = msg->data[i];
        const float angle = msg->data[i + 1];

        if (!std::isfinite(distance) || !std::isfinite(angle)) {
            continue;
        }

        if (distance < nearest_distance) {
            nearest_distance = distance;
            nearest_angle = angle;
            found_valid_pair = true;
        }
    }

    if (found_valid_pair) {
        latest_obstacle_angle_deg = static_cast<double>(nearest_angle);
        obstacle_angle_valid = true;
    } else {
        obstacle_angle_valid = false;
    }
}

void controlCallback(const std_msgs::Float32MultiArray::ConstPtr& array_msg) {
    if (array_msg->data.size() > 2) {
        command_received = (static_cast<int>(array_msg->data[2]) != 0);
    }
}

#endif

#ifdef LOG_ENCODER_MODE
// Encoder mode callbacks
void velopubCallback(const std_msgs::Float64::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    latest_velopub = msg;
}

void v1Callback(const std_msgs::UInt16::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    latest_v1 = msg;
}

void speedCallback(const std_msgs::Float64::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(log_mutex);
    latest_speed = msg;
}
#endif

int main(int argc, char** argv) {
    ros::init(argc, argv, "central_logging_node");
    ros::NodeHandle nh;

    std::time_t now = std::time(nullptr);
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "combinedlog_%Y-%m-%d_%H:%M:%S.csv", std::localtime(&now));
    filename = buffer;

    log_file.open(filename, std::ios::out);
    if (!log_file.is_open()) {
        ROS_FATAL("Failed to create log file: %s", filename.c_str());
        return -1;
    }

#ifdef LOG_FNAV_MODE
    // FNAV mode header
    log_file << "Timestamp (UTC),"
             << "fnav (m/s²),"
             << "Distance (m),"
             << "Obstacle Angle (deg),"
             << "D Stable (m),"
             << "Velocity Command (m/s),"
             << "Real Velocity (m/s),"
             << "A Dyn (m),"
             << "Obstacle Speed From Lidar (m/s),"
             << "Gain (K),"
             << "SFM Enable\n";
             
    // FNAV mode subscribers
    ros::Subscriber sub_fnav = nh.subscribe("/fnav", 10, fnavCallback);
    ros::Subscriber sub_distance = nh.subscribe("/near_distance", 10, distanceCallback);
    ros::Subscriber sub_distance_filtered = nh.subscribe("/distance_filtered", 10, filteredDistanceCallback);
    ros::Subscriber sub_d_stable = nh.subscribe("/d_stable", 10, stableDistanceCallback);
    ros::Subscriber sub_velocity_cmd = nh.subscribe("/velopub", 10, velocityCmdCallback);
    ros::Subscriber sub_real_velocity = nh.subscribe("/vehicle_speed", 10, realVelocityCallback);
    ros::Subscriber sub_a_dyn = nh.subscribe("/a_dyn", 10, aDynCallback);
    ros::Subscriber sub_obstacle_speed_from_lidar = nh.subscribe("/obstacle_speed_from_lidar", 10, obstacleSpeedFromLidarCallback);
    ros::Subscriber sub_fuzzy = nh.subscribe("/fuzzy", 10, fuzzy_callback);
    ros::Subscriber sub_obstacle_angle = nh.subscribe("/distances_minus150_to_150", 10, obstacleAngleCallback);
    ros::Subscriber sub = nh.subscribe("/control_array", 10, controlCallback);
#endif

#ifdef LOG_ENCODER_MODE
    // Encoder mode header
    log_file << "Timestamp (UTC),"
             << "Vehicle Speed (m/s),"
             << "Encoder Count,"
             << "Velocity Command (m/s)\n";
             
    // Encoder mode subscribers
    ros::Subscriber sub_velo = nh.subscribe("velopub", 10, velopubCallback);
    ros::Subscriber sub_v1 = nh.subscribe("v1", 10, v1Callback);
    ros::Subscriber sub_speed = nh.subscribe("vehicle_speed", 10, speedCallback);
#endif

    ROS_INFO("Logging to: %s", filename.c_str());
    ros::Rate rate(10);

    while (ros::ok()) {
        ros::spinOnce();
        std::lock_guard<std::mutex> lock(log_mutex);

        log_file << getROSTimestamp();

#ifdef LOG_FNAV_MODE
        // FNAV mode logging
        log_file << (latest_fnav ? "," + std::to_string(latest_fnav->x) : ",");
        if (latest_distance_filtered) {
            log_file << "," + std::to_string(latest_distance_filtered->data);
        } else if (latest_distance && latest_distance->data.size() >= 1) {
            // Fallback for older SFM nodes that don't publish /distance_filtered
            log_file << "," + std::to_string(latest_distance->data[0]);
        } else {
            log_file << ",";
        }
        log_file << (obstacle_angle_valid ? "," + std::to_string(latest_obstacle_angle_deg) : ",");
        log_file << (latest_d_stable ? "," + std::to_string(latest_d_stable->data) : ",");
        log_file << (latest_velocity_cmd ? "," + std::to_string(latest_velocity_cmd->data) : ",");
        log_file << (latest_real_velocity ? "," + std::to_string(latest_real_velocity->data) : ",");
        log_file << (latest_a_dyn ? "," + std::to_string(latest_a_dyn->data) : ",");
        log_file << (latest_obstacle_speed_from_lidar ? "," + std::to_string(latest_obstacle_speed_from_lidar->data) : ",");
        log_file << (fuzzy_gain ? "," + std::to_string(fuzzy_gain->data) : "");
        log_file << (command_received ? "," + std::to_string(command_received) : ",0");
#endif

#ifdef LOG_ENCODER_MODE
        // Encoder mode logging
        log_file << (latest_velopub ? "," + std::to_string(latest_velopub->data) : ",");
        log_file << (latest_v1 ? "," + std::to_string(latest_v1->data) : ",");
        log_file << (latest_speed ? "," + std::to_string(latest_speed->data) : "");
#endif

        log_file << "\n";
        rate.sleep();
    }

    log_file.close();
    return 0;
}


// #include <ros/ros.h>
// #include <std_msgs/Float32.h>
// #include <std_msgs/Float64.h>
// #include <std_msgs/UInt16.h>
// #include <std_msgs/Float32MultiArray.h>
// #include <fstream>
// #include <iomanip>
// #include <sstream>
// #include <ctime>
// #include <mutex>
// #include <limits>
// #include <cmath>

// // Define your logging mode (uncomment one)
// #define LOG_FNAV_MODE
// // #define LOG_ENCODER_MODE

// #ifdef LOG_FNAV_MODE
// #include <geometry_msgs/Point.h>
// #endif

// std::ofstream log_file;
// std::string filename;
// std::mutex log_mutex;
// bool command_received = false;
// double latest_obstacle_angle_deg = 0.0;
// bool obstacle_angle_valid = false;

// #ifdef LOG_FNAV_MODE
// // FNAV mode variables
// std_msgs::Float64::ConstPtr latest_velocity_cmd;
// std_msgs::Float32MultiArray::ConstPtr latest_distance;
// std_msgs::Float32::ConstPtr latest_distance_filtered;
// std_msgs::Float32::ConstPtr latest_d_stable;
// std_msgs::Float32::ConstPtr fuzzy_gain;
// geometry_msgs::Point::ConstPtr latest_fnav;
// std_msgs::Float64::ConstPtr latest_real_velocity;
// #endif

// #ifdef LOG_ENCODER_MODE
// // Encoder mode variables
// std_msgs::Float64::ConstPtr latest_velopub;
// std_msgs::UInt16::ConstPtr latest_v1;
// std_msgs::Float64::ConstPtr latest_speed;
// #endif

// std::string getROSTimestamp() {
//     ros::Time now = ros::Time::now();
//     std::time_t seconds = now.sec;
//     std::tm* time_info = std::localtime(&seconds);
//     std::ostringstream timestamp;
//     timestamp << std::put_time(time_info, "%Y-%m-%d %H:%M:%S");
//     timestamp << "." << std::setw(9) << std::setfill('0') << now.nsec;
//     return timestamp.str();
// }

// #ifdef LOG_FNAV_MODE
// // FNAV mode callbacks
// void fnavCallback(const geometry_msgs::Point::ConstPtr& msg) {
//     std::lock_guard<std::mutex> lock(log_mutex);
//     latest_fnav = msg;
// }

// void distanceCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
//     std::lock_guard<std::mutex> lock(log_mutex);
//     latest_distance = msg;
// }

// void filteredDistanceCallback(const std_msgs::Float32::ConstPtr& msg) {
//     std::lock_guard<std::mutex> lock(log_mutex);
//     latest_distance_filtered = msg;
// }

// void stableDistanceCallback(const std_msgs::Float32::ConstPtr& msg) {
//     std::lock_guard<std::mutex> lock(log_mutex);
//     latest_d_stable = msg;
// }

// void velocityCmdCallback(const std_msgs::Float64::ConstPtr& msg) {
//     std::lock_guard<std::mutex> lock(log_mutex);
//     latest_velocity_cmd = msg;
// }

// void realVelocityCallback(const std_msgs::Float64::ConstPtr& msg) {
//     std::lock_guard<std::mutex> lock(log_mutex);
//     latest_real_velocity = msg;
// }

// void fuzzy_callback(const std_msgs::Float32::ConstPtr& msg){
//     std::lock_guard<std::mutex> lock(log_mutex);
//     fuzzy_gain = msg;
// }

// void obstacleAngleCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
//     std::lock_guard<std::mutex> lock(log_mutex);

//     if (msg->data.size() < 2) {
//         obstacle_angle_valid = false;
//         return;
//     }

//     float nearest_distance = std::numeric_limits<float>::infinity();
//     float nearest_angle = 0.0f;
//     bool found_valid_pair = false;

//     // Expected format: [distance_0, angle_0, distance_1, angle_1, ...]
//     for (size_t i = 0; i + 1 < msg->data.size(); i += 2) {
//         const float distance = msg->data[i];
//         const float angle = msg->data[i + 1];

//         if (!std::isfinite(distance) || !std::isfinite(angle)) {
//             continue;
//         }

//         if (distance < nearest_distance) {
//             nearest_distance = distance;
//             nearest_angle = angle;
//             found_valid_pair = true;
//         }
//     }

//     if (found_valid_pair) {
//         latest_obstacle_angle_deg = static_cast<double>(nearest_angle);
//         obstacle_angle_valid = true;
//     } else {
//         obstacle_angle_valid = false;
//     }
// }

// void controlCallback(const std_msgs::Float32MultiArray::ConstPtr& array_msg) {
//     if (array_msg->data.size() > 2) {
//         command_received = (static_cast<int>(array_msg->data[2]) != 0);
//     }
// }

// #endif

// #ifdef LOG_ENCODER_MODE
// // Encoder mode callbacks
// void velopubCallback(const std_msgs::Float64::ConstPtr& msg) {
//     std::lock_guard<std::mutex> lock(log_mutex);
//     latest_velopub = msg;
// }

// void v1Callback(const std_msgs::UInt16::ConstPtr& msg) {
//     std::lock_guard<std::mutex> lock(log_mutex);
//     latest_v1 = msg;
// }

// void speedCallback(const std_msgs::Float64::ConstPtr& msg) {
//     std::lock_guard<std::mutex> lock(log_mutex);
//     latest_speed = msg;
// }
// #endif

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "central_logging_node");
//     ros::NodeHandle nh;

//     std::time_t now = std::time(nullptr);
//     char buffer[80];
//     std::strftime(buffer, sizeof(buffer), "combinedlog_%Y-%m-%d_%H:%M:%S.csv", std::localtime(&now));
//     filename = buffer;

//     log_file.open(filename, std::ios::out);
//     if (!log_file.is_open()) {
//         ROS_FATAL("Failed to create log file: %s", filename.c_str());
//         return -1;
//     }

// #ifdef LOG_FNAV_MODE
//     // FNAV mode header
//     log_file << "Timestamp (UTC),"
//              << "fnav (m/sÂ²),"
//              << "Distance (m),"
//              << "Obstacle Angle (deg),"
//              << "D Stable (m),"
//              << "Velocity Command (m/s),"
//              << "Real Velocity (m/s),"
//              << "Gain (K),"
//              << "SFM Enable\n";
             
//     // FNAV mode subscribers
//     ros::Subscriber sub_fnav = nh.subscribe("/fnav", 10, fnavCallback);
//     ros::Subscriber sub_distance = nh.subscribe("/near_distance", 10, distanceCallback);
//     ros::Subscriber sub_distance_filtered = nh.subscribe("/distance_filtered", 10, filteredDistanceCallback);
//     ros::Subscriber sub_d_stable = nh.subscribe("/d_stable", 10, stableDistanceCallback);
//     ros::Subscriber sub_velocity_cmd = nh.subscribe("/velopub", 10, velocityCmdCallback);
//     ros::Subscriber sub_real_velocity = nh.subscribe("/vehicle_speed", 10, realVelocityCallback);
//     ros::Subscriber sub_fuzzy = nh.subscribe("/fuzzy", 10, fuzzy_callback);
//     ros::Subscriber sub_obstacle_angle = nh.subscribe("/distances_minus150_to_150", 10, obstacleAngleCallback);
//     ros::Subscriber sub = nh.subscribe("/control_array", 10, controlCallback);
// #endif

// #ifdef LOG_ENCODER_MODE
//     // Encoder mode header
//     log_file << "Timestamp (UTC),"
//              << "Vehicle Speed (m/s),"
//              << "Encoder Count,"
//              << "Velocity Command (m/s)\n";
             
//     // Encoder mode subscribers
//     ros::Subscriber sub_velo = nh.subscribe("velopub", 10, velopubCallback);
//     ros::Subscriber sub_v1 = nh.subscribe("v1", 10, v1Callback);
//     ros::Subscriber sub_speed = nh.subscribe("vehicle_speed", 10, speedCallback);
// #endif

//     ROS_INFO("Logging to: %s", filename.c_str());
//     ros::Rate rate(10);

//     while (ros::ok()) {
//         ros::spinOnce();
//         std::lock_guard<std::mutex> lock(log_mutex);

//         log_file << getROSTimestamp();

// #ifdef LOG_FNAV_MODE
//         // FNAV mode logging
//         log_file << (latest_fnav ? "," + std::to_string(latest_fnav->x) : ",");
//         if (latest_distance_filtered) {
//             log_file << "," + std::to_string(latest_distance_filtered->data);
//         } else if (latest_distance && latest_distance->data.size() >= 1) {
//             // Fallback for older SFM nodes that don't publish /distance_filtered
//             log_file << "," + std::to_string(latest_distance->data[0]);
//         } else {
//             log_file << ",";
//         }
//         log_file << (obstacle_angle_valid ? "," + std::to_string(latest_obstacle_angle_deg) : ",");
//         log_file << (latest_d_stable ? "," + std::to_string(latest_d_stable->data) : ",");
//         log_file << (latest_velocity_cmd ? "," + std::to_string(latest_velocity_cmd->data) : ",");
//         log_file << (latest_real_velocity ? "," + std::to_string(latest_real_velocity->data) : ",");
//         log_file << (fuzzy_gain ? "," + std::to_string(fuzzy_gain->data) : "");
//         log_file << (command_received ? "," + std::to_string(command_received) : ",0");
// #endif

// #ifdef LOG_ENCODER_MODE
//         // Encoder mode logging
//         log_file << (latest_velopub ? "," + std::to_string(latest_velopub->data) : ",");
//         log_file << (latest_v1 ? "," + std::to_string(latest_v1->data) : ",");
//         log_file << (latest_speed ? "," + std::to_string(latest_speed->data) : "");
// #endif

//         log_file << "\n";
//         rate.sleep();
//     }

//     log_file.close();
//     return 0;
// }