#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <cmath>
#include <vector>

// Helper function to calculate the magnitude of a vector
double magnitude(const std::vector<double>& v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1]);
}

// Helper function to add two vectors
std::vector<double> add(const std::vector<double>& v1, const std::vector<double>& v2) {
    return {v1[0] + v2[0], v1[1] + v2[1]};
}

// Helper function to subtract two vectors
std::vector<double> subtract(const std::vector<double>& v1, const std::vector<double>& v2) {
    return {v1[0] - v2[0], v1[1] - v2[1]};
}

// Helper function to scale a vector
std::vector<double> scale(const std::vector<double>& v, double scalar) {
    return {v[0] * scalar, v[1] * scalar};
}

// Calculate goal force f_goal
std::vector<double> calculateGoalForce(const std::vector<double>& desiredVelocity, const std::vector<double>& currentVelocity) {
    double mass = 1.0;  // Mass of the agent
    double tau = 0.5;   // Characteristic time parameter
    std::vector<double> velocityDifference = subtract(desiredVelocity, currentVelocity);
    return scale(velocityDifference, mass / tau);
}

// Calculate the static social force
std::vector<double> calculateSocialStaticForce(const std::vector<double>& robotPosition, const std::vector<double>& obstaclePosition, double k, double r_R, double Psi, double omega) {
    std::vector<double> direction = subtract(robotPosition, obstaclePosition);
    double d_Rh = magnitude(direction);

    if (d_Rh >= r_R) {
        return {0, 0};  // Force ignored if distance is larger than radius
    }

    std::vector<double> e_Rh = scale(direction, 1.0 / d_Rh); // Normalize direction
    double forceMagnitude = k * std::exp((r_R - d_Rh) / Psi) * omega;

    return scale(e_Rh, forceMagnitude);
}

// Calculate the physical static force
std::vector<double> calculatePhysicalStaticForce(const std::vector<double>& robotPosition, const std::vector<double>& obstaclePosition, double k, double r_R) {
    std::vector<double> direction = subtract(robotPosition, obstaclePosition);
    double d_Rh = magnitude(direction);

    if (d_Rh >= r_R) {
        return {0, 0};  // Force ignored if distance is larger than radius
    }

    std::vector<double> e_Rh = scale(direction, 1.0 / d_Rh); // Normalize direction
    double forceMagnitude = k * (r_R - d_Rh);

    return scale(e_Rh, forceMagnitude);
}

// Combine the social and physical static forces
std::vector<double> calculateStaticForce(const std::vector<double>& robotPosition, const std::vector<double>& obstaclePosition, double k, double r_R, double Psi, double omega) {
    std::vector<double> socialForce = calculateSocialStaticForce(robotPosition, obstaclePosition, k, r_R, Psi, omega);
    std::vector<double> physicalForce = calculatePhysicalStaticForce(robotPosition, obstaclePosition, k, r_R);

    return add(socialForce, physicalForce);
}

// Calculate the resultant navigation force f_nav
std::vector<double> calculateNavigationForce(const std::vector<double>& currentPos, const std::vector<double>& goalPos, 
                                  const std::vector<std::vector<double>>& staticObstacles, 
                                  const std::vector<std::vector<double>>& dynamicObstacles) {
    std::vector<double> f_goal = calculateGoalForce(currentPos, goalPos);

    std::vector<double> f_static = {0, 0};
    for (const auto& obstacle : staticObstacles) {
        f_static = add(f_static, calculateStaticForce(currentPos, obstacle, K_static, 0.5, psi, 1.0));
    }

    std::vector<double> f_dynamic = {0, 0};
    for (const auto& obstacle : dynamicObstacles) {
        f_dynamic = add(f_dynamic, calculateStaticForce(currentPos, obstacle, K_dynamic, 0.5, psi, 1.0));
    }

    return add(add(f_goal, f_static), f_dynamic);  // Navigation force f_nav is the sum of all forces
}

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
    // Example positions and obstacles
    std::vector<double> robotPos = {0.0, 0.0};
    std::vector<double> goalPos = {5.0, 5.0};

    // Define some static and dynamic obstacles
    std::vector<std::vector<double>> staticObstacles = {{2.0, 3.0}, {-1.0, 1.0}};
    std::vector<std::vector<double>> dynamicObstacles = {{4.0, 2.0}, {1.5, -0.5}};

    // Calculate the navigation force
    std::vector<double> f_nav = calculateNavigationForce(robotPos, goalPos, staticObstacles, dynamicObstacles);

    // Output the navigation force
    std::cout << "Navigation Force (f_nav): x = " << f_nav[0] << ", y = " << f_nav[1] << std::endl;
    std::cout << "Magnitude of f_nav: " << magnitude(f_nav) << std::endl;

    return 0;
}



















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
#include <chrono>
#include <ctime>
#include <fstream>
#include <cstring> // For strerror
#include <sstream>

ros::Publisher magnitude_publisher;
ros::Publisher direction_publisher;
ros::Publisher vector_y_publisher;
ros::Publisher vector_x_publisher;

std::ofstream log_file;
std::string filename; // Declare globally or pass to functions
std::vector<std::pair<float, float>> distances_90_to_180;
std::vector<std::pair<float, float>> distances_minus180_to_minus90;
std::vector<std::pair<float, float>> distances_minus150_to_150;

float v0 = 0.8;
float m = 1;
float dt = 0.001;
float resultant_magnitude = 0.0;
float resultant_angle = 0.0;

void distances90to180Callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    distances_90_to_180.clear();
    
    if (msg->data.size() % 2 != 0) {
        ROS_WARN("Received data size is not even. Skipping update.");
        return;
    }

    for (size_t i = 0; i < msg->data.size(); i += 2) {
        float distance = msg->data[i];
        float angle = msg->data[i + 1];
        distances_90_to_180.emplace_back(distance, angle);
    }
}

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

void distancesMinus180toMinus90Callback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    distances_minus180_to_minus90.clear();

    if (msg->data.size() % 2 != 0) {
        ROS_WARN("Received data size for distances_minus180_to_minus90 is not even. Skipping update.");
        return;
    }

    for (size_t i = 0; i < msg->data.size(); i += 2) {
        float distance = msg->data[i];
        float angle = msg->data[i + 1];
        distances_minus180_to_minus90.emplace_back(distance, angle);
    }
}

float calculateCoordinates(float degree) {
    float a = 1; 
    float b = 0.5;  

    float angle = degree * M_PI / 180.0;

    float r = (a * b) / std::sqrt(std::pow(b * std::cos(angle), 2) + std::pow(a * std::sin(angle), 2));

    float x = r * std::cos(angle);
    float y = r * std::sin(angle);

    float distance = std::sqrt(x * x + y * y);
    
    return distance;
}

float forceSocial(float distance, float degree) {
    float k = 1.0;
    float psi = 0.20;

    float r_value = calculateCoordinates(degree);
    float force = k * std::exp((r_value - distance) / psi);

    return force;
}

float forcePhysical(float distance, float degree) {
    float k = 1.0;  

    float r_value = calculateCoordinates(degree);
    float force = k * (r_value - distance);

    return force;
}

float invforceSocial(float distance, float degree) {
    float k = 1.0;
    float psi = 0.20;

    float r_value = calculateCoordinates(degree);
    float force = k / std::exp((r_value - distance) / psi);  // Inverted behavior

    return force;
}

float invforcePhysical(float distance, float degree) {
    float k = 1.0;

    float r_value = calculateCoordinates(degree);
    float force = k / (r_value - distance + 1);  // Inverted behavior with safeguard

    return force;
}

std::string getROSTimestamp() {
    // Get the current ROS time
    ros::Time now = ros::Time::now();

    // Convert ROS time to seconds
    std::time_t seconds = now.sec;

    // Convert to standard time structure
    std::tm* time_info = std::localtime(&seconds);

    // Format the time string
    std::ostringstream timestamp;
    timestamp << std::put_time(time_info, "%Y-%m-%d %H:%M:%S"); // Format: YYYY-MM-DD HH:MM:SS

    // Add nanoseconds for full ROS time precision
    timestamp << "." << std::setw(9) << std::setfill('0') << now.nsec;

    return timestamp.str();
}

void sfm() {
    // Open log file in append mode
    log_file.open(filename, std::ios::out | std::ios::app);
    if (!log_file.is_open()) {
        ROS_ERROR("Failed to open log file");
        ROS_INFO("FAILED OPENNING: %s", filename.c_str());
        return;
    }
    
    auto nearest_90_to_180 = std::min_element(distances_90_to_180.begin(), distances_90_to_180.end(),
                                                [](const std::pair<float, float>& a, const std::pair<float, float>& b) {
                                                    return a.first < b.first;
                                                });

    auto nearest_minus180_to_minus90 = std::min_element(distances_minus180_to_minus90.begin(), distances_minus180_to_minus90.end(),
                                                            [](const std::pair<float, float>& a, const std::pair<float, float>& b) {
                                                                return a.first < b.first;
                                                            });

    auto nearest_minus150_to_150 = std::min_element(distances_minus150_to_150.begin(), distances_minus150_to_150.end(),
                                                        [](const std::pair<float, float>& a, const std::pair<float, float>& b) {
                                                            return a.first < b.first;
                                                        });                           

    std::vector<float> distances = {2.0}; 
    std::vector<float> angles = {180.0}; 

    // std::time_t now = std::time(nullptr);
    // std::string timestamp = std::ctime(&now);
    // timestamp.pop_back(); // Remove trailing newline

    std::string timestamp = getROSTimestamp();

    log_file << timestamp << ",";
    log_file << resultant_magnitude << ",";
    log_file << resultant_angle << ",";

    if (!distances_minus150_to_150.empty()) {
        float nearest_distance_minus150_to_150 = nearest_minus150_to_150->first;
        float nearest_angle_minus150_to_150 = nearest_minus150_to_150->second;
        log_file << nearest_distance_minus150_to_150 << ",";
        log_file << nearest_angle_minus150_to_150 << ",";

        float x_minus150_to_150 = nearest_distance_minus150_to_150 * cos(nearest_angle_minus150_to_150 * M_PI / 180.0);
        float y_minus150_to_150 = nearest_distance_minus150_to_150 * sin(nearest_angle_minus150_to_150 * M_PI / 180.0);

        float f_social = forceSocial(nearest_distance_minus150_to_150, nearest_angle_minus150_to_150);
        float f_physical = forcePhysical(nearest_distance_minus150_to_150, nearest_angle_minus150_to_150);
        float f_force_static = f_social + f_physical;
        float f_angle_reverse = nearest_angle_minus150_to_150 * -1;

        std::cout << "Obstacle depan (degree): " << nearest_angle_minus150_to_150 << ", Distance: " << nearest_distance_minus150_to_150 << " m\n";

        distances.push_back(f_force_static);
        angles.push_back(f_angle_reverse);
    }
    else{
        log_file << "No Data"<< ",";
        log_file << "No Data"<< ",";
    }


    if (!distances_90_to_180.empty()) {
        float nearest_distance_90_to_180 = nearest_90_to_180->first;
        float nearest_angle_90_to_180 = nearest_90_to_180->second;
        log_file << nearest_distance_90_to_180 << ",";
        log_file << nearest_angle_90_to_180 << ",";

        float x_90_to_180 = nearest_distance_90_to_180 * cos(nearest_angle_90_to_180 * M_PI / 180.0);
        float y_90_to_180 = nearest_distance_90_to_180 * sin(nearest_angle_90_to_180 * M_PI / 180.0);

        float sosial = forceSocial(nearest_distance_90_to_180, nearest_angle_90_to_180);
        float physical = forcePhysical(nearest_distance_90_to_180, nearest_angle_90_to_180);
        float force_static = sosial + physical;
        float angle_reverse = nearest_angle_90_to_180 * -1;

        std::cout << "Obstacle kanan (degree): " << nearest_angle_90_to_180 << ", Distance: " << nearest_distance_90_to_180 << " m\n";

        distances.push_back(force_static);
        angles.push_back(angle_reverse);
    }
    else{
        log_file << "No Data"<< ",";
        log_file << "No Data"<< ",";
    }


    if (!distances_minus180_to_minus90.empty()) {
        float nearest_distance_minus180_to_minus90 = nearest_minus180_to_minus90->first;
        float nearest_angle_minus180_to_minus90 = nearest_minus180_to_minus90->second;
        log_file << nearest_distance_minus180_to_minus90 << ",";
        log_file << nearest_angle_minus180_to_minus90 << ",";

        float x_minus180_to_minus90 = nearest_distance_minus180_to_minus90 * cos(nearest_angle_minus180_to_minus90 * M_PI / 180.0);
        float y_minus180_to_minus90 = nearest_distance_minus180_to_minus90 * sin(nearest_angle_minus180_to_minus90 * M_PI / 180.0);

        float rsosial = forceSocial(nearest_distance_minus180_to_minus90, nearest_angle_minus180_to_minus90);
        float rphysical = forcePhysical(nearest_distance_minus180_to_minus90, nearest_angle_minus180_to_minus90);
        float r_force_static = rsosial + rphysical;
        float r_angle_reverse = nearest_angle_minus180_to_minus90 * -1;

        std::cout << "Obstacle kiri (degree): " << nearest_angle_minus180_to_minus90 << ", Distance: " << nearest_distance_minus180_to_minus90 << " m\n";

        distances.push_back(r_force_static);
        angles.push_back(r_angle_reverse);
    }
    else{
        log_file << "No Data"<< ",";
        log_file << "No Data"<< ",";
    }
    log_file << "\n";


    std::cout << "Dis : ";
    for (float dist : distances) std::cout << dist << " ";
    std::cout << "\nAng : ";
    for (float ang : angles) std::cout << ang << " ";
    std::cout << std::endl;

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

    std::cout << "Fnav : " << resultant_magnitude << std::endl;
    std::cout << "Rho : " << resultant_angle << std::endl;

    std_msgs::Float32 xmsg;
    xmsg.data = resultant_vector_x;
    vector_x_publisher.publish(xmsg);

    std_msgs::Float32 ymsg;
    ymsg.data = resultant_vector_y;
    vector_y_publisher.publish(ymsg);

    geometry_msgs::Point magnitude_msg;
    magnitude_msg.x = resultant_magnitude;
    magnitude_msg.y = 0;
    magnitude_msg.z = 0;
    magnitude_publisher.publish(magnitude_msg);

    geometry_msgs::Point direction_msg;
    direction_msg.x = resultant_angle;
    direction_msg.y = 0;
    direction_msg.z = 0;
    direction_publisher.publish(direction_msg);

    log_file.close();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sfmtest");
    ros::NodeHandle n;

    magnitude_publisher = n.advertise<geometry_msgs::Point>("/fnav", 1000);
    direction_publisher = n.advertise<geometry_msgs::Point>("/rho", 1000);
    vector_y_publisher = n.advertise<std_msgs::Float32>("/resultant_vector_y", 1000);
    vector_x_publisher = n.advertise<std_msgs::Float32>("/resultant_vector_x", 1000);

    ros::Subscriber sub_distances_90_to_180 = n.subscribe("/distances_90_to_180", 1000, distances90to180Callback);
    ros::Subscriber sub_distances_minus180_to_minus90 = n.subscribe("/distances_minus180_to_minus90", 1000, distancesMinus180toMinus90Callback);
    ros::Subscriber sub_distances_minus150_to_150 = n.subscribe("/distances_minus150_to_150", 1000, distancesMinus150to150Callback);

    // Generate a unique filename based on the current time
    std::time_t now = std::time(nullptr);
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "topic_log_%Y%m%d_%H%M%S.csv", std::localtime(&now));
    filename = buffer;

    log_file.open(filename, std::ios::out);
    if (log_file.is_open()) {
        ROS_INFO("OPENNING: %s", filename.c_str());
        log_file << "Timestamp,Magnitude,Direction,Distances90to180,Angle90to180,Distances-180to-90,Angle-180to-90,Distances-150to150,Angle-150to150\n"; // Header
        log_file.close();
    } else {
        ROS_ERROR("Failed to create log file");
        return 1;
    }
    
    ros::Rate loop_rate(100);

    while (ros::ok()) {
        sfm();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}





//drive.cpp
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <algorithm> // For std::min and std::max

// PID Controller Class
class PIDController {
public:
    PIDController(double Kp, double Ki, double Kd)
        : Kp(Kp), Ki(Ki), Kd(Kd), prev_error(0), integral(0) {}
    double compute(double setpoint, double measurement, double dt) {
        double error = setpoint - measurement;
        integral += error * dt;
        double derivative = (error - prev_error) / dt;
        double output = Kp * error + Ki * integral + Kd * derivative;
        prev_error = error;
        return output;
    }
private:
    double Kp, Ki, Kd;
    double prev_error;
    double integral;
};

// Control Class
class Control {
public:
    Control(ros::NodeHandle& nh)
        : v0(1.8), m(2.6), dt(0.1), vtt(100), current_turn(2050),
          pid(1.6, 0.02, 0.075) {
        // Publishers
        pub1 = nh.advertise<std_msgs::UInt16>("v1", 10);
        pub3 = nh.advertise<std_msgs::Float64>("velocity1", 10);
        pub4 = nh.advertise<std_msgs::Float64>("velocity2", 10);

        // Subscribers
        joy_sub = nh.subscribe("/joy", 10, &Control::joystick_sensor_callback, this);
        rho_sub = nh.subscribe("/rho", 10, &Control::steering_callback, this);
        fnav_sub = nh.subscribe("/fnav", 10, &Control::distance_callback, this);
    }

    static double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    static double clamp(double value, double min_value, double max_value) {
        return std::max(std::min(value, max_value), min_value);
    }

    void joystick_sensor_callback(const sensor_msgs::Joy::ConstPtr& msg) {
        const auto& axes = msg->axes;
        const auto& buttons = msg->buttons;

        double getLeftYAxis = map(axes[1], -1.0, 1.0, 50.0, 130.0);  // pwm
        double getBackward = map(getLeftYAxis, 255, 0, 90, 20);
        double getForward = map(getLeftYAxis, 255, 510, 90, 170);

        double setPoint = 2040;
        double servoDegree = 250;
        double getRightXAxis = map(axes[3], -1.0, 1.0, 0.0, 510.0);  // velocity sudut
        double getRight = map(getRightXAxis, 255, 0, setPoint, setPoint + servoDegree);
        double getLeft = map(getRightXAxis, 255, 510, setPoint, setPoint - servoDegree);

        std_msgs::UInt16 v1_msg;
        v1_msg.data = static_cast<uint16_t>(getLeftYAxis);
        pub1.publish(v1_msg);

        std_msgs::Float64 velocity1_msg, velocity2_msg;
        velocity1_msg.data = getRight;
        velocity2_msg.data = getLeft;
        pub3.publish(velocity1_msg);
        pub4.publish(velocity2_msg);
    }

    void steering_callback(const geometry_msgs::Point::ConstPtr& msg) {
        double setPoint = 2050;
        double angle = msg->x;
        double step_size = 150;

        if (angle < -180 || angle > 180) {
            ROS_WARN("Angle out of range: %f", angle);
            return;
        }

        double target_turn;
        if (angle >= -180 && angle <= -150) {
            double feedback = map(angle, -160, -180, 250.0, 0);
            target_turn = setPoint - feedback;
        } else if (angle >= -160 && angle <= -90) {
            target_turn = 1800;  // Full left
        } else if (angle > 150 && angle <= 180) {
            double feedback = map(angle, 160, 180, 250.0, 0);
            target_turn = setPoint + feedback;
        } else if (angle > 90 && angle <= 160) {
            target_turn = 2300;  // Full right
        } else {
            target_turn = setPoint;
        }

        // Ensure target_turn is within limits
        target_turn = clamp(target_turn, 1800, 2300);

        // Smooth transition to target_turn
        if (current_turn < target_turn) {
            current_turn = std::min(current_turn + step_size, target_turn);
        } else if (current_turn > target_turn) {
            current_turn = std::max(current_turn - step_size, target_turn);
        }

        ROS_INFO("angle: %f, turn: %f", angle, current_turn);

        std_msgs::Float64 turn_msg;
        turn_msg.data = current_turn;
        pub4.publish(turn_msg);
    }

    void distance_callback(const geometry_msgs::Point::ConstPtr& msg) {
        double fnav = msg->x;
        double velocity = v0 - (fnav / m) * dt;
        if (velocity >= 3) {
            velocity = 3;
        }

        int velocity_now = velmap(velocity, 0.3, 3, 100, 115);

        std_msgs::UInt16 vtt_msg;
        vtt = 107;
        vtt_msg.data = vtt;
        pub1.publish(vtt_msg);

        ROS_INFO("v_now: %d", velocity_now);
    }

    void loop() {
        ros::Rate rate(50); // Set the loop rate to 50 Hz (adjust as needed)
        while (ros::ok()) {
            // Check if the node is shutting down
            if (ros::isShuttingDown()) {
                shutdown_hook();
                break;
            }

            // Add any periodic tasks here if needed
            ros::spinOnce(); // Process callbacks
            rate.sleep();    // Sleep to maintain the desired loop rate
        }
    }

    // Shutdown hook to reset values
    void shutdown_hook() {
        ROS_INFO("Shutting down. Resetting v1 to 0 and velocity2 to 2000.");

        // Publish v1 = 0
        std_msgs::UInt16 v1_msg;
        v1_msg.data = 0;
        pub1.publish(v1_msg);

        // Publish velocity2 = 2000
        std_msgs::Float64 velocity2_msg;
        velocity2_msg.data = 2000.0;
        pub4.publish(velocity2_msg);

        ROS_INFO("Values reset successfully.");
    }

private:
    double v0, m, dt;
    int vtt;
    double current_turn;

    PIDController pid;

    ros::Publisher pub1, pub3, pub4;
    ros::Subscriber joy_sub, rho_sub, fnav_sub;

    static int velmap(double x, double in_min, double in_max, double out_min, double out_max) {
        return static_cast<int>((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "joystick_sensor_subscriber");
    ros::NodeHandle nh;

    Control control(nh);

    // Start the main loop
    control.loop();

    return 0;
}



/////////

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <cmath>
#include <vector>

// Helper function to calculate the magnitude of a vector
double magnitude(const std::vector<double>& v) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1]);
}

// Helper function to add two vectors
std::vector<double> add(const std::vector<double>& v1, const std::vector<double>& v2) {
    return {v1[0] + v2[0], v1[1] + v2[1]};
}

// Helper function to subtract two vectors
std::vector<double> subtract(const std::vector<double>& v1, const std::vector<double>& v2) {
    return {v1[0] - v2[0], v1[1] - v2[1]};
}

// Helper function to scale a vector
std::vector<double> scale(const std::vector<double>& v, double scalar) {
    return {v[0] * scalar, v[1] * scalar};
}

// Calculate goal force f_goal
std::vector<double> calculateGoalForce(const std::vector<double>& desiredVelocity, const std::vector<double>& currentVelocity) {
    double mass = 1.0;  // Mass of the agent
    double tau = 0.5;   // Characteristic time parameter
    std::vector<double> velocityDifference = subtract(desiredVelocity, currentVelocity);
    return scale(velocityDifference, mass / tau);
}

// Calculate the static social force
std::vector<double> calculateSocialStaticForce(const std::vector<double>& robotPosition, const std::vector<double>& obstaclePosition, double k, double r_R, double Psi, double omega) {
    std::vector<double> direction = subtract(robotPosition, obstaclePosition);
    double d_Rh = magnitude(direction);

    if (d_Rh >= r_R) {
        return {0, 0};  // Force ignored if distance is larger than radius
    }

    std::vector<double> e_Rh = scale(direction, 1.0 / d_Rh); // Normalize direction
    double forceMagnitude = k * std::exp((r_R - d_Rh) / Psi) * omega;

    return scale(e_Rh, forceMagnitude);
}

// Calculate the physical static force
std::vector<double> calculatePhysicalStaticForce(const std::vector<double>& robotPosition, const std::vector<double>& obstaclePosition, double k, double r_R) {
    std::vector<double> direction = subtract(robotPosition, obstaclePosition);
    double d_Rh = magnitude(direction);

    if (d_Rh >= r_R) {
        return {0, 0};  // Force ignored if distance is larger than radius
    }

    std::vector<double> e_Rh = scale(direction, 1.0 / d_Rh); // Normalize direction
    double forceMagnitude = k * (r_R - d_Rh);

    return scale(e_Rh, forceMagnitude);
}

// Combine the social and physical static forces
std::vector<double> calculateStaticForce(const std::vector<double>& robotPosition, const std::vector<double>& obstaclePosition, double k, double r_R, double Psi, double omega) {
    std::vector<double> socialForce = calculateSocialStaticForce(robotPosition, obstaclePosition, k, r_R, Psi, omega);
    std::vector<double> physicalForce = calculatePhysicalStaticForce(robotPosition, obstaclePosition, k, r_R);

    return add(socialForce, physicalForce);
}

// Calculate the resultant navigation force f_nav
std::vector<double> calculateNavigationForce(const std::vector<double>& currentPos, const std::vector<double>& goalPos, 
                                  const std::vector<std::vector<double>>& staticObstacles, 
                                  const std::vector<std::vector<double>>& dynamicObstacles) {
    std::vector<double> f_goal = calculateGoalForce(currentPos, goalPos);

    std::vector<double> f_static = {0, 0};
    for (const auto& obstacle : staticObstacles) {
        f_static = add(f_static, calculateStaticForce(currentPos, obstacle, K_static, 0.5, psi, 1.0));
    }

    std::vector<double> f_dynamic = {0, 0};
    for (const auto& obstacle : dynamicObstacles) {
        f_dynamic = add(f_dynamic, calculateStaticForce(currentPos, obstacle, K_dynamic, 0.5, psi, 1.0));
    }

    return add(add(f_goal, f_static), f_dynamic);  // Navigation force f_nav is the sum of all forces
}

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "talker");
	ros::NodeHandle n;
    // Example positions and obstacles
    std::vector<double> robotPos = {0.0, 0.0};
    std::vector<double> goalPos = {5.0, 5.0};

    // Define some static and dynamic obstacles
    std::vector<std::vector<double>> staticObstacles = {{2.0, 3.0}, {-1.0, 1.0}};
    std::vector<std::vector<double>> dynamicObstacles = {{4.0, 2.0}, {1.5, -0.5}};

    // Calculate the navigation force
    std::vector<double> f_nav = calculateNavigationForce(robotPos, goalPos, staticObstacles, dynamicObstacles);

    // Output the navigation force
    std::cout << "Navigation Force (f_nav): x = " << f_nav[0] << ", y = " << f_nav[1] << std::endl;
    std::cout << "Magnitude of f_nav: " << magnitude(f_nav) << std::endl;

    return 0;
}

void steering_callback(const geometry_msgs::Point::ConstPtr& msg) {
    double setPoint = 2050;
    double angle = msg->x;
    double step_size = 150;

    if (angle < -180 || angle > 180) {
        ROS_WARN("Angle out of range: %f", angle);
        return;
    }

    double target_turn;
    if (angle >= -180 && angle <= -150) {
        double feedback = map(angle, -160, -180, 250.0, 0);
        target_turn = setPoint - feedback;
    } else if (angle >= -160 && angle <= -90) {
        target_turn = 1800;  // Full left
    } else if (angle > 150 && angle <= 180) {
        double feedback = map(angle, 160, 180, 250.0, 0);
        target_turn = setPoint + feedback;
    } else if (angle > 90 && angle <= 160) {
        target_turn = 2300;  // Full right
    } else {
        target_turn = setPoint;
    }

    target_turn = clamp(target_turn, 1800, 2300);

    if (current_turn < target_turn) {
        current_turn = std::min(current_turn + step_size, target_turn);
    } else if (current_turn > target_turn) {
        current_turn = std::max(current_turn - step_size, target_turn);
    }

    std_msgs::Float64 turn_msg;
    turn_msg.data = manual_steer;
    pub4.publish(turn_msg);
}