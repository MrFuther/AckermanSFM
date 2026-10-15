#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>

class LaneKeepingNode {
private:
    ros::NodeHandle nh;
    ros::Subscriber image_sub;
    ros::Publisher error_pub;
    ros::Publisher lka_img_pub;
    ros::Publisher lka_mask_pub;

    float prev_center_x = 320.0; 
    float alpha_filter = 0.2;    // Faktor kehalusan (Semakin kecil = semakin halus/lambat)
    int center_x = 320;       
    int target_y = 300;          // Lookahead distance (sudah pas)
    // int assumed_lane_width = 350;  // PERUBAHAN
    float last_known_width = 350.0;

    int getXIntercept(const std::vector<cv::Point>& pts, int y_target) {
        if (pts.size() < 2) return -1;
        cv::Vec4f line;
        cv::fitLine(pts, line, cv::DIST_L2, 0, 0.01, 0.01);
        float vx = line[0], vy = line[1], x0 = line[2], y0 = line[3];
        if (vy == 0) return x0;
        return std::round(x0 + (vx / vy) * (y_target - y0));
    }

public:
    LaneKeepingNode() {
        image_sub = nh.subscribe("/camera", 1, &LaneKeepingNode::imageCallback, this);
        error_pub = nh.advertise<std_msgs::Float32>("/lane_error", 10);
        lka_img_pub = nh.advertise<sensor_msgs::Image>("/lka/output_image", 1);
        lka_mask_pub = nh.advertise<sensor_msgs::Image>("/lka/mask_image", 1);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat frame = cv_ptr->image.clone();

            // 1. ROI & Masking
            cv::Mat mask = cv::Mat::zeros(frame.size(), CV_8UC1);
            cv::Point pts[4] = { 
                cv::Point(0, 480),   
                cv::Point(640, 480), 
                cv::Point(640, 120), 
                cv::Point(0, 120)    
            };
            cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255));

            // 2. Filter HLS untuk warna putih 
            // (Catatan: Jika masih banyak pantulan lantai, naikkan nilai 183 menjadi 200 atau 210)
            cv::Mat hls, white_mask;
            cv::cvtColor(frame, hls, cv::COLOR_BGR2HLS);
            cv::inRange(hls, cv::Scalar(0, 183, 0), cv::Scalar(180, 255, 255), white_mask);
            cv::bitwise_and(white_mask, mask, white_mask);

            // 3. Deteksi Tepi & Hough Transform
            cv::Mat edges;
            cv::Canny(white_mask, edges, 50, 150);
            std::vector<cv::Vec4i> lines;
            cv::HoughLinesP(edges, lines, 1, CV_PI/180, 40, 40, 20);

            // 4. Pemisahan Garis
            std::vector<cv::Point> left_pts, right_pts;
            for (auto l : lines) {
                float slope = (float)(l[3] - l[1]) / (float)(l[2] - l[0] + 0.0001);
                if (slope < -0.3 && slope > -3.0) { 
                    left_pts.push_back(cv::Point(l[0], l[1])); left_pts.push_back(cv::Point(l[2], l[3]));
                } else if (slope > 0.3 && slope < 3.0) { 
                    right_pts.push_back(cv::Point(l[0], l[1])); right_pts.push_back(cv::Point(l[2], l[3]));
                }
            }

            // 5. Kalkulasi Titik Tengah Mentah (ADAPTIF)
            int left_x = getXIntercept(left_pts, target_y);
            int right_x = getXIntercept(right_pts, target_y);
            float raw_lane_center = prev_center_x; 

            if (left_x != -1 && right_x != -1) {
                int current_width = right_x - left_x;
                if (current_width > 150 && current_width < 800){
                    raw_lane_center = (left_x + right_x) / 2.0;
                    
                    // Update memori lebar jalan secara dinamis (EMA 10% data baru, 90% data lama)
                    last_known_width = (0.1 * current_width) + (0.9 * last_known_width);
                }
            } else if (left_x != -1) {
                // Gunakan memori lebar jalan terakhir jika marka kanan hilang
                raw_lane_center = left_x + (last_known_width / 2.0);
            } else if (right_x != -1) {
                // Gunakan memori lebar jalan terakhir jika marka kiri hilang
                raw_lane_center = right_x - (last_known_width / 2.0);
            }

            // // 5. Kalkulasi Titik Tengah Mentah
            // int left_x = getXIntercept(left_pts, target_y);
            // int right_x = getXIntercept(right_pts, target_y);
            // float raw_lane_center = prev_center_x; // Gunakan float untuk perhitungan EMA

            // if (left_x != -1 && right_x != -1) {
            //     int width = right_x - left_x;
            //     if (width > 150 && width < 800){
            //         raw_lane_center = (left_x + right_x) / 2.0;
            //     }
            // } else if (left_x != -1) {
            //     raw_lane_center = left_x + 250.0;
            // } else if (right_x != -1) {
            //     raw_lane_center = right_x - 250.0;
            // }

            // ========================================================
            // EKSEKUSI FILTER EMA (Meredam Jitter Titik Hijau)
            // ========================================================
            float smoothed_lane_center = (alpha_filter * raw_lane_center) + ((1.0 - alpha_filter) * prev_center_x);
            prev_center_x = smoothed_lane_center; // Simpan untuk frame selanjutnya

            // 6. PUBLISH ERROR YANG SUDAH DIHALUSKAN
            float error = smoothed_lane_center - center_x; 
            float deadzone = 30.0f; // Batas zona mati dalam piksel
            
            if (std::abs(error) < deadzone) {
                error = 0.0f;
            } else {
                if (error > 0) {
                    error -= deadzone;
                } else {
                    error += deadzone;
                }
            }
            
            std_msgs::Float32 error_msg;
            error_msg.data = error;
            error_pub.publish(error_msg);

            // Visualisasi
            cv::line(frame, cv::Point(center_x, 0), cv::Point(center_x, 480), cv::Scalar(255, 0, 0), 1); 
            
            // Gambar titik hijau menggunakan koordinat yang sudah dihaluskan
            cv::circle(frame, cv::Point(smoothed_lane_center, target_y), 8, cv::Scalar(0, 255, 0), -1); 
            
            // cv::imshow("LKA Mask", white_mask);
            // cv::imshow("LKA Output", frame);
            // cv::waitKey(1);
            
            sensor_msgs::ImagePtr msg_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            lka_img_pub.publish(msg_out);

            sensor_msgs::ImagePtr msg_mask = cv_bridge::CvImage(std_msgs::Header(), "mono8", white_mask).toImageMsg();
            lka_mask_pub.publish(msg_mask);

        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "lane_keeping_node");
    LaneKeepingNode lkn;
    ROS_INFO("LKA Node Active with EMA Filter. Publishing error to /lane_error");
    ros::spin();
    return 0;
}