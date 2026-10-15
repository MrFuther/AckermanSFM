// ########################################################################################################
// ####################                          PROGRAM DEMO                          #################### 
// ########################################################################################################

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <fstream>

ros::Publisher image_sync_pub;
ros::Publisher scan_sync_pub;
ros::Time start_time;

void callback(const sensor_msgs::ImageConstPtr& image, const sensor_msgs::LaserScanConstPtr& scan) {
    double relative_image_time = (image->header.stamp - start_time).toSec() + 1.0;
    double relative_scan_time = (scan->header.stamp - start_time).toSec() + 1.0;

    image_sync_pub.publish(image);
    scan_sync_pub.publish(scan);

    // Tampilkan informasi timestamp relatif di terminal
    // ROS_INFO(" ");
    // ROS_INFO("Relative Image timestamp: %.6f", relative_image_time);
    // ROS_INFO("Relative Scan timestamp: %.6f", relative_scan_time);

    // Simpan data ke file CSV
    // std::ofstream csv_file;
    // csv_file.open("output.csv", std::ios::app);
    // if (csv_file.is_open()) {
    //     csv_file << relative_image_time << "," << relative_scan_time << "\n";
    //     csv_file.close();
    // } else {
    //     ROS_ERROR("Gagal membuka file CSV!");
    // }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "time_sync_node");
    ros::NodeHandle nh;

    start_time = ros::Time::now();

    image_sync_pub = nh.advertise<sensor_msgs::Image>("/camera_sync", 10);
    scan_sync_pub = nh.advertise<sensor_msgs::LaserScan>("/scan_sync", 10);

    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera", 10);
    message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, "/scan", 10);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::LaserScan> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, scan_sub);

    sync.registerCallback(boost::bind(&callback, _1, _2));

    ros::spin();
    return 0;
}


























// ########################################################################################################
// ####################             PROGRAM UJI COBA DAN PENGAMBILAN DATA              #################### 
// ########################################################################################################



// ======================================= SINKRONISASI WAKTU 3x ==========================================

// #include <ros/ros.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/LaserScan.h>
// #include <message_filters/subscriber.h>
// #include <deque>
// #include <mutex>
// #include <fstream>
// #include <sstream>

// ros::Publisher image_sync_pub;
// ros::Publisher scan_sync_pub;
// ros::Time start_time;

// // Buffer untuk menyimpan data lidar
// std::deque<sensor_msgs::LaserScanConstPtr> scan_buffer;
// std::mutex buffer_mutex;

// const size_t MAX_BUFFER_SIZE = 10; // Maksimal 10 frame lidar disimpan

// // File output CSV
// std::ofstream csv_file;

// void scanCallback(const sensor_msgs::LaserScanConstPtr& scan) {
//     std::lock_guard<std::mutex> lock(buffer_mutex);
//     scan_buffer.push_back(scan);

//     // Batasi ukuran buffer
//     if (scan_buffer.size() > MAX_BUFFER_SIZE) {
//         scan_buffer.pop_front();
//     }
// }

// void imageCallback(const sensor_msgs::ImageConstPtr& image) {
//     std::lock_guard<std::mutex> lock(buffer_mutex);

//     if (!scan_buffer.empty()) {
//         // Cari lidar dengan timestamp paling dekat ke timestamp kamera
//         ros::Time img_time = image->header.stamp;
//         double min_diff = std::numeric_limits<double>::max();
//         sensor_msgs::LaserScanConstPtr best_scan;

//         for (const auto& scan : scan_buffer) {
//             double diff = fabs((img_time - scan->header.stamp).toSec());
//             if (diff < min_diff) {
//                 min_diff = diff;
//                 best_scan = scan;
//             }
//         }

//         if (best_scan) {
//             double relative_image_time = (image->header.stamp - start_time).toSec() + 1.0;
//             double relative_scan_time = (best_scan->header.stamp - start_time).toSec() + 1.0;

//             image_sync_pub.publish(image);
//             scan_sync_pub.publish(best_scan);

//             ROS_INFO("Scan  |  Image  =  %.6f  |  %.6f  ", relative_scan_time, relative_image_time);

//             // Menulis ke file CSV
//             //csv_file << relative_scan_time << "," << relative_image_time << std::endl;
//         }
//     }
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "manual_sync_node");
//     ros::NodeHandle nh;

//     start_time = ros::Time::now();

//     // Buka file CSV untuk menyimpan data sinkronisasi
//     // csv_file.open("/home/jetson/ACKERMAN/sync_data.csv"); // Sesuaikan path sesuai sistem Anda
//     // if (!csv_file.is_open()) {
//     //     ROS_ERROR("Gagal membuka file CSV untuk penyimpanan data.");
//     //     return -1;
//     // }

//     // Tuliskan header CSV
//     //csv_file << "Scan,Image" << std::endl;


//     image_sync_pub = nh.advertise<sensor_msgs::Image>("/camera_sync", 10);
//     scan_sync_pub = nh.advertise<sensor_msgs::LaserScan>("/scan_sync", 10);

//     // Subscriber untuk lidar
//     message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, "/scan", 10);
//     scan_sub.registerCallback(&scanCallback);

//     // Subscriber untuk kamera
//     message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera", 10);
//     image_sub.registerCallback(&imageCallback);

//     ros::spin();

//     // Tutup file saat program selesai
//     // if (csv_file.is_open()) {
//     //     csv_file.close();
//     //     ROS_INFO("File CSV berhasil ditutup.");
//     // }

//     return 0;
// }