#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <chrono>
#include <string>
#include <fstream> 
#include <iomanip> 
#include <sstream>
#include <deque>  
#include <numeric>

const double distance_per_count = 0.00198412698; // meters per count

// const size_t moving_average_window = 10; // Adjust this value (e.g., 5-20)
// std::deque<double> speed_buffer;         // Buffer to store recent speeds

// double filtered_speed = 0.0;       // Initialize filtered speed
// const double alpha = 0.2;          // Smoothing factor

// Logging variables
// std::ofstream log_file;
// std::string filename;

// // Function to generate ROS timestamp string
// std::string getROSTimestamp() {
//     ros::Time now = ros::Time::now();
//     std::time_t seconds = now.sec;
//     std::tm* time_info = std::localtime(&seconds);
//     std::ostringstream timestamp;
//     timestamp << std::put_time(time_info, "%Y-%m-%d %H:%M:%S");
//     timestamp << "." << std::setw(9) << std::setfill('0') << now.nsec;
//     return timestamp.str();
// }

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "encoder_speed_node");
    ros::NodeHandle nh;
    ros::Publisher speed_pub = nh.advertise<std_msgs::Float64>("/vehicle_speed", 10);

    int fd = open("/dev/blackpill", O_RDWR | O_NOCTTY);
    if (fd == -1) {
        ROS_ERROR("Failed to open serial port");
        return -1;
    }

    struct termios toptions;
    tcgetattr(fd, &toptions);
    cfsetispeed(&toptions, B115200);
    cfsetospeed(&toptions, B115200);
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    toptions.c_lflag |= ICANON;
    toptions.c_cc[VEOL] = '\n';
    tcsetattr(fd, TCSANOW, &toptions);

    int prev_count = 0;
    ros::Time last_time = ros::Time::now();
    std::string receive_buffer;
    char prev_buf[64] = "";

    while(ros::ok()) {
        // Trigger data transmission
        write(fd, "0", 1);

        char buf[64];
        int n = read(fd, buf, sizeof(buf)-1);
        if (n > 0) {
            buf[n] = '\0';
            receive_buffer += buf; // Append new data to buffer
                     
            // if (strcmp(buf, prev_buf) != 0) {
            //     printf("%s", buf);  // Print only new data
            //     strncpy(prev_buf, buf, sizeof(prev_buf));
            //     prev_buf[sizeof(prev_buf)-1] = '\0';  // Ensure termination
            // }

            size_t newline_pos;

            while ((newline_pos = receive_buffer.find('\n')) != std::string::npos) {
                std::string line = receive_buffer.substr(0, newline_pos);
                receive_buffer = receive_buffer.substr(newline_pos + 1);

                float speed;
                int delta_count;
                char sep1, sep2;

                std::stringstream ss(line);
                if (ss >> speed >> sep1 >> delta_count >> sep2 && sep1 == ';' && sep2 == ';') {
                    // Get current time for delta_time calculation
                    // auto current_time = std::chrono::steady_clock::now();
                    // auto delta_time = std::chrono::duration<double>(current_time - prev_time).count();

                    // Publish ROS message
                    std_msgs::Float64 speed_msg;
                    speed_msg.data = speed;
                    speed_pub.publish(speed_msg);

                    // Update previous time
                    // prev_time = current_time;

                    // Print the parsed values
                    // ROS_INFO("Speed = %.8f, Delta Count = %d", speed, delta_count);
                }
                else {
                    ROS_WARN("Failed to parse line: %s", line.c_str());
                }
            
                // float current_count = atoi(line.c_str());

                // if (current_count != prev_count) {
                //     // ros::Time current_time = ros::Time::now();
                //     // ros::Duration delta_time = current_time - last_time;
                //     // double dt = delta_time.toSec();
                //     // int delta_count = current_count - prev_count;
                //     // double speed = (delta_count * distance_per_count) / dt;

                //     // Apply low-pass filter
                //     // if (filtered_speed == 0.0) {
                //     //     filtered_speed = speed; // Initialize on first run
                //     // } else {
                //     //     filtered_speed = alpha * speed + (1.0 - alpha) * filtered_speed;
                //     // }
                //     // filtered_speed = speed;

                //     // // Apply moving average filter
                //     // speed_buffer.push_back(speed);
                //     // if (speed_buffer.size() > moving_average_window) {
                //     //     speed_buffer.pop_front(); // Keep buffer size fixed
                //     // }

                //     // double filtered_speed = std::accumulate(speed_buffer.begin(), speed_buffer.end(), 0.0) / speed_buffer.size();

                //     // Publish ROS message
                //     // std_msgs::Float64 speed_msg;
                //     // speed_msg.data = speed;
                //     // speed_pub.publish(speed_msg);

                //     // ROS_INFO("Speed = %.8f", speed);
                //     // // ROS_INFO("Filtered Speed = %.8f", filtered_speed);
                //     // ROS_INFO("dt = %f", dt);
                //     ROS_INFO("cc = %f", current_count);
                //     // ROS_INFO("dc = %d \n", delta_count);

                //     // last_time = current_time;

                //     // Log to file
                //     // log_file << getROSTimestamp() << ","
                //     //          << std::fixed << std::setprecision(8) << speed << ","
                //     //          << delta_count << ","
                //     //          << delta_time << ","
                //     //          << distance_per_count << "\n";

                //     // Update previous values
                //     // prev_count = current_count;
                //     // prev_time = current_time;
                // }
            }
        }

        ros::spinOnce();
    }
    close(fd);
    return 0;
}