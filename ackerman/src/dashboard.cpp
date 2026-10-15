#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <mutex>

// Variabel penampung gambar dari thread callback
cv::Mat lka_img, mask_img, yolo_img;
std::mutex mtx;

void lkaCallback(const sensor_msgs::ImageConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mtx);
    try { lka_img = cv_bridge::toCvCopy(msg, "bgr8")->image; } catch (...) {}
}

void maskCallback(const sensor_msgs::ImageConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mtx);
    try { 
        cv::Mat temp = cv_bridge::toCvCopy(msg, "mono8")->image; 
        cv::cvtColor(temp, mask_img, cv::COLOR_GRAY2BGR); // Ubah ke 3-channel agar bisa dijahit
    } catch (...) {}
}

void yoloCallback(const sensor_msgs::ImageConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mtx);
    try { yolo_img = cv_bridge::toCvCopy(msg, "bgr8")->image; } catch (...) {}
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dashboard_node");
    ros::NodeHandle nh;

    // Subscribe ke semua gambar hasil komputasi
    ros::Subscriber sub_lka = nh.subscribe("/lka/output_image", 1, lkaCallback);
    ros::Subscriber sub_mask = nh.subscribe("/lka/mask_image", 1, maskCallback);
    ros::Subscriber sub_yolo = nh.subscribe("/yolo/output_image", 1, yoloCallback);

    ROS_INFO("ADAS Dashboard Visualizer Active!");

    ros::Rate rate(30); // Refresh rate layar
    while (ros::ok()) {
        ros::spinOnce();
        
        cv::Mat img1, img2, img3;
        {
            std::lock_guard<std::mutex> lock(mtx);
            if (!lka_img.empty()) img1 = lka_img.clone();
            if (!mask_img.empty()) img2 = mask_img.clone();
            if (!yolo_img.empty()) img3 = yolo_img.clone();
        }

        // Buat kanvas kosong jika node asal belum mengirim gambar
        if (img1.empty()) img1 = cv::Mat::zeros(480, 640, CV_8UC3);
        if (img2.empty()) img2 = cv::Mat::zeros(480, 640, CV_8UC3);
        if (img3.empty()) img3 = cv::Mat::zeros(480, 640, CV_8UC3);
        cv::Mat blank = cv::Mat::zeros(480, 640, CV_8UC3);

        // Samakan ukuran untuk Grid
        cv::resize(img1, img1, cv::Size(640, 480));
        cv::resize(img2, img2, cv::Size(640, 480));
        cv::resize(img3, img3, cv::Size(640, 480));

        // Beri Label Teks
        cv::putText(img1, "1. LKA Camera", cv::Point(15, 35), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        cv::putText(img2, "2. LKA Mask", cv::Point(15, 35), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        cv::putText(img3, "3. YOLO + LiDAR", cv::Point(15, 35), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
        cv::putText(blank, "System Standby", cv::Point(15, 35), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(100, 100, 100), 2);

        // Menjahit Matriks (Grid 2x2)
        cv::Mat top_row, bottom_row, dashboard;
        cv::hconcat(img1, img2, top_row);       // Baris atas
        cv::hconcat(img3, blank, bottom_row);   // Baris bawah (kanan bawah sengaja dikosongkan)
        cv::vconcat(top_row, bottom_row, dashboard);

        // Scale down agar pas di layar monitor
        cv::resize(dashboard, dashboard, cv::Size(), 0.6, 0.6); 

        cv::imshow("Autonomous Driving Dashboard", dashboard);
        cv::waitKey(1);

        rate.sleep();
    }
    return 0;
}