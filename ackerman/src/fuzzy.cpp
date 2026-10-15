//FUZZY sebelum Optimasi

// #include <stdio.h>
// #include <math.h>
// #include <ros/ros.h>
// #include <std_msgs/Float32MultiArray.h>
// #include <std_msgs/Float64.h>
// #include <std_msgs/Float32.h>

// /*IF YOU WANT TO TEST THE FUZZY JUST UNCOMMENT THE DEFINE BELOW DAWG*/
// // #define TESTING

// float speed = 0;  
// float distance = 0;

// /*==== Membership function Speed Set START ====*/
// float speed_membership_low(float x) {
//     if (x < 0.0 || x > 1.5) return 0.0;
//     return (1.5 - x) / 1.5;
// }

// float speed_membership_medium(float x) {
//     if (x < 0.3 || x > 2.7) return 0.0;
//     if (x <= 1.5) return (x - 0.3) / (1.5 - 0.3);
//     else return (2.7 - x) / (2.7 - 1.5);          
// }

// float speed_membership_high(float x) {
//     if (x < 1.5 || x > 3.0) return 0.0;
//     return (x - 1.5) / 1.5;
// }
// /*==== Membership function Speed Set END ====*/

// /*==== Membership function Distance START ====*/
// float distance_membership_close(float x) {
//     if (x < 0.0 || x > 1.5) return 0.0;
//     return (1.5 - x) / 1.5;
// }

// float distance_membership_medium(float x) {
//     if (x < 0.3 || x > 2.7) return 0.0;
//     if (x <= 1.5) return (x - 0.3) / (1.5 - 0.3);
//     else return (2.7 - x) / (2.7 - 1.5); 
// }

// float distance_membership_far(float x) {
//     if (x < 1.5 || x > 3.0) return 0.0;
//     return (x - 1.5) / 1.5;
// }
// /*==== Membership function Distance END ====*/

// /*==== Membership function output (-1 to 1) START ====*/
// float acceleration_membership_negative(float x) {
//     if (x < 0 || x > 0.5) return 0;
//     return (0.5 - x) / 0.5;
// }

// float acceleration_membership_zero(float x) {
//     if (x < 0.2 || x > 0.8) return 0;
//     if (x <= 0.5)
//         return (x - 0.2) / 0.3;
//     else
//         return (0.8 - x) / 0.3;
// }

// float acceleration_membership_positive(float x) {
//     if (x < 0.5 || x > 1) return 0;
//     return (x - 0.5) / 0.5;
// }
// /*==== Membership function output END ====*/

// // Centroid defuzzification
// float defuzzify(float neg_strength, float zero_strength, float pos_strength) {
//     float numerator = 0.0;
//     float denominator = 0.0;
//     const float STEP = 0.01;
//     const float OUTPUT_MIN = 0.0;
//     const float OUTPUT_MAX = 1.0;

//     for (float x = OUTPUT_MIN; x <= OUTPUT_MAX; x += STEP) {
//         float neg_mf = acceleration_membership_negative(x) * neg_strength;
//         float zero_mf = acceleration_membership_zero(x) * zero_strength;
//         float pos_mf = acceleration_membership_positive(x) * pos_strength;

//         float total = neg_mf + zero_mf + pos_mf;
//         numerator += x * total;
//         denominator += total;
//     }

//     return (denominator == 0.0) ? 0.0 : numerator / denominator;
// }

// // Fuzzy Inference System
// float fuzzy_controller(float speed, float distance) {
//     // Fuzzify inputs
//     float low_speed = speed_membership_low(speed);
//     float medium_speed = speed_membership_medium(speed);
//     float high_speed = speed_membership_high(speed);

//     float close_distance = distance_membership_close(distance);
//     float medium_distance = distance_membership_medium(distance);
//     float far_distance = distance_membership_far(distance);

//     float acc_neg = 0.0;   // Decelerate
//     float acc_zero = 0.0;  // Constant
//     float acc_pos = 0.0;   // Accelerate

//     // Rule 1: Close & Fast → Decelerate
//     float r1 = fmin(close_distance, high_speed);
//     acc_pos = fmax(acc_pos, r1);

//     // Rule 2: Mid & Mid → Constant
//     float r2 = fmin(medium_distance, medium_speed);
//     acc_zero = fmax(acc_zero, r2);

//     // Rule 3: Far & Medium → Accelerate
//     float r3 = fmin(far_distance, low_speed);
//     acc_neg = fmax(acc_neg, r3);

//     // Rule 4: Close & Mid → Decelerate
//     float r4 = fmin(close_distance, medium_speed);
//     acc_pos = fmax(acc_pos, r4);

//     // Rule 5: Close & Slow → Constant
//     float r5 = fmin(close_distance, low_speed);
//     acc_pos = fmax(acc_pos, r5);

//     // Rule 6: Mid & Fast → Decelerate
//     float r6 = fmin(medium_distance, high_speed);
//     acc_pos = fmax(acc_pos, r6);

//     // Rule 7: Mid & Mid → Constant
//     float r7 = fmin(medium_distance, medium_speed);
//     acc_zero = fmax(acc_zero, r7);

//     // Rule 8: Far & Slow → Accelerate
//     float r8 = fmin(far_distance, medium_speed);
//     acc_neg = fmax(acc_neg, r8);

//     // Rule 9: Far & Fast → Constant
//     float r9 = fmin(far_distance, high_speed);
//     acc_neg = fmax(acc_neg, r9);

//     return defuzzify(acc_neg, acc_zero, acc_pos);
// }

// #ifdef TESTING
// void controlCallback(const std_msgs::Float32MultiArray::ConstPtr& array_msg) {
//     float up_down_value = array_msg->data[4];
//     float left_right_value = array_msg->data[5];

//     if(up_down_value != 0){
//         if(left_right_value == 1){
//             distance_to_car = distance_to_car + (up_down_value);
//         }
//         else if(left_right_value == 2){
//             speed_setpoint = speed_setpoint + (up_down_value);
//         }
//         else if(left_right_value == 3){
//             //Do nothing
//         }
//         float acceleration = fuzzy_controller(speed, distance);

//         printf("Distance: %.2f \n", distance_to_car);
//         printf("Speed set: %.2f \n", speed_setpoint);
//         printf("Acceleration/Deceleration: %.2f \n", acceleration);
//     }    
    
// }
// #endif

// void distanceCallback(const std_msgs::Float32::ConstPtr& msg) {
//     distance = msg->data;
// }

// void speed_encoder_callback(const std_msgs::Float64::ConstPtr& msg) {
//     speed = static_cast<float>(msg->data);
//     if(speed < 0){
//         speed = speed * -1;
//     }
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "fuzzy_logic");
//     ros::NodeHandle nh;

//     ros::Publisher pub = nh.advertise<std_msgs::Float32>("/fuzzy", 10);

//     #ifdef TESTING
//     ros::Subscriber sub = nh.subscribe("/control_array", 10, controlCallback);
//     #endif

//     ros::Subscriber sub_distance = nh.subscribe("/near_distance", 10, distanceCallback);
//     ros::Subscriber speed_encoder_sub = nh.subscribe("/vehicle_speed", 10, speed_encoder_callback);

//     ros::Rate rate(30);
//     while (ros::ok()) {
//         float acceleration = fuzzy_controller(speed, distance);

//         // printf("Distance: %.2f \n", distance);
//         // printf("Speed set: %.2f \n", speed);
//         // printf("Acceleration/Deceleration: %.2f \n", acceleration);

//         std_msgs::Float32 gain;
//         gain.data = acceleration; 
//         // gain.data = 1; 
//         pub.publish(gain);

//         ros::spinOnce(); 
//         rate.sleep();
//     }

//     return 0;
// }

// //fuzzy sugeno
// #include <stdio.h>
// #include <math.h>
// #include <ros/ros.h>
// #include <std_msgs/Float32MultiArray.h>
// #include <std_msgs/Float64.h>
// #include <std_msgs/Float32.h>

// float speed = 0.0;  
// float distance = 0.0; 

// /*==== Membership function Speed Set START ====*/
// float speed_membership_low(float x) {
//     if (x < 0.0 || x > 1.0) return 0.0;
//     return (1.0 - x) / 1.0;
// }

// float speed_membership_medium(float x) {
//     if (x < 0.5 || x > 1.5) return 0.0;
//     if (x <= 1.0) return (x - 0.5) / (1.0 - 0.5);
//     else return (1.5 - x) / (1.5 - 1.0);
// }

// float speed_membership_high(float x) {
//     if (x < 1.0 || x > 2.0) return 0.0;
//     return (x - 1.0) / 1.0;
// }

// /*==== Membership function Distance START ====*/
// float distance_membership_intimate(float x) {
//     if (x < 0.0 || x > 0.5) {
//         return 0.0;
//     }
//     return (0.5 - x) / 0.5;
// }

// float distance_membership_personal(float x) {
//     if (x < 0.0 || x > 1.2) {
//         return 0.0;
//     }
//     if (x <= 0.5) {
//         return x / 0.5;
//     } else {
//         return (1.2 - x) / (1.2 - 0.5); 
//     }
// }

// float distance_membership_social(float x) {
//     if (x < 0.5 || x > 3.5) {
//         return 0.0;
//     }
//     if (x <= 1.2) {
//         return (x - 0.5) / (1.2 - 0.5);
//     } else {
//         return (3.5 - x) / (3.5 - 1.2);
//     }
// }

// float distance_membership_public(float x) {
//     if (x <= 1.2) {
//         return 0.0;
//     }
//     if (x >= 3.5) {
//         return 1.0;
//     }
//     return (x - 1.2) / (3.5 - 1.2);
// }
// /*==== Membership function Distance END ====*/

// /*==== Membership function output (0 to 1) START ====*/
// float calculate_fuzzy_output(float current_speed, float current_distance) {
//     // === Langkah 1: Fuzzifikasi ===
//     float mu_speed_low = speed_membership_low(current_speed);
//     float mu_speed_medium = speed_membership_medium(current_speed);
//     float mu_speed_high = speed_membership_high(current_speed);

//     float mu_dist_intimate = distance_membership_intimate(current_distance);
//     float mu_dist_personal = distance_membership_personal(current_distance);
//     float mu_dist_social = distance_membership_social(current_distance);
//     float mu_dist_public = distance_membership_public(current_distance);

//     // === Langkah 2: Evaluasi Aturan (Inference) ===
//     // Aturan untuk Speed = Slow
//     float w1 = fminf(mu_speed_low, mu_dist_intimate);   
//     float w2 = fminf(mu_speed_low, mu_dist_personal);  
//     float w3 = fminf(mu_speed_low, mu_dist_social);   
//     float w4 = fminf(mu_speed_low, mu_dist_public);   

//     // Aturan untuk Speed = Medium
//     float w5 = fminf(mu_speed_medium, mu_dist_intimate);  
//     float w6 = fminf(mu_speed_medium, mu_dist_personal);
//     float w7 = fminf(mu_speed_medium, mu_dist_social);  
//     float w8 = fminf(mu_speed_medium, mu_dist_public); 

//     // Aturan untuk Speed = Fast
//     float w9 = fminf(mu_speed_high, mu_dist_intimate);  
//     float w10 = fminf(mu_speed_high, mu_dist_personal); 
//     float w11 = fminf(mu_speed_high, mu_dist_social);   
//     float w12 = fminf(mu_speed_high, mu_dist_public);   

//     // === Langkah 3: Defuzzifikasi (Weighted Average) ===
//     // Menghitung keluaran crisp. Rumus: Σ(wi * zi) / Σ(wi)
    

//     //1 proxemic 1
//     // float numerator = (w1 * 0.827619) + (w2 * 0.373844) + (w3 * 0.334608) + (w4 * 0.645339) +
//     //                   (w5 * 0.430783) + (w6 * 0.673313) + (w7 * 0.387826) + (w8 * 0.255102) +
//     //                   (w9 * 0.332552) + (w10 * 0.706367) + (w11 * 0.340372) + (w12 * 0.339520);

//     // float denominator = w1 + w2 + w3 + w4 + w5 + w6 + w7 + w8 + w9 + w10 + w11 + w12;


//     //2 proxemic 2
//     // float numerator = (w1 * 0.213593) + (w2 * 0.664020) + (w3 * 0.572206) + (w4 * 0.410200) +
//     //                   (w5 * 0.066365) + (w6 * 0.020712) + (w7 * 0.629395) + (w8 * 0.432469) +
//     //                   (w9 * 0.846017) + (w10 * 0.865536) + (w11 * 0.535803) + (w12 * 0.355519);

//     // float denominator = w1 + w2 + w3 + w4 + w5 + w6 + w7 + w8 + w9 + w10 + w11 + w12;

//     //proxemic 1.5
//     // float numerator = (w1 * 0.476846) + (w2 * 0.582222) + (w3 * 0.726270) + (w4 * 0.447533) +
//     //                   (w5 * 0.871495) + (w6 * 0.488910) + (w7 * 0.610521) + (w8 * 0.247337) +
//     //                   (w9 * 0.583452) + (w10 * 0.666644) + (w11 * 0.319747) + (w12 * 0.188717);

//     // float denominator = w1 + w2 + w3 + w4 + w5 + w6 + w7 + w8 + w9 + w10 + w11 + w12;

//     //proxemic 1.2
//     float numerator = (w1 * 0.699225) + (w2 * 0.694988) + (w3 * 0.366987) + (w4 * 0.038210) +
//                       (w5 * 0.648455) + (w6 * 0.196733) + (w7 * 0.310280) + (w8 * 0.545767) +
//                       (w9 * 0.815789) + (w10 * 0.676523) + (w11 * 0.275246) + (w12 * 0.212974);

//     float denominator = w1 + w2 + w3 + w4 + w5 + w6 + w7 + w8 + w9 + w10 + w11 + w12;

//     // Menghindari pembagian dengan nol jika tidak ada aturan yang aktif
//     if (denominator == 0) {
//         return 0.0;
//     }

//     return numerator / denominator;
// }

// void distanceCallback(const std_msgs::Float32::ConstPtr& msg) {
//     distance = msg->data;
//     ROS_DEBUG("Fuzzy received distance: %.2f", distance);
// }

// void speed_encoder_callback(const std_msgs::Float64::ConstPtr& msg) {
//     speed = static_cast<float>(msg->data);
//     if(speed < 0){
//         speed = speed * -1;
//     }
//     ROS_DEBUG("Fuzzy received speed: %.2f", speed);
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "fuzzy_controller");
//     ros::NodeHandle nh;

//     // Publisher - Output to SFM node
//     ros::Publisher fuzzy_pub = nh.advertise<std_msgs::Float32>("/fuzzy", 10);

//     // Subscribers - Input from SFM and encoder nodes
//     ros::Subscriber sub_distance = nh.subscribe("/near_distance", 10, distanceCallback);
//     ros::Subscriber speed_encoder_sub = nh.subscribe("/vehicle_speed", 10, speed_encoder_callback);
//     ros::Rate rate(30); // 30 Hz sama seperti package ackerman
    
//     while (ros::ok()) {
//         float acceleration = calculate_fuzzy_output(speed, distance);

//         ROS_DEBUG("Distance: %.2f, Speed: %.2f, Fuzzy Gain: %.2f", distance, speed, acceleration);
        
//         // Publish fuzzy gain to SFM node
//         std_msgs::Float32 gain;
//         gain.data = acceleration; 
//         fuzzy_pub.publish(gain);

//         ros::spinOnce(); 
//         rate.sleep();
//     }

//     return 0;
// }



//fuzzy sugeno ganti input
//fuzzy sugeno
#include <stdio.h>
#include <math.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>

// Precalculated constants for optimization
const float EPSILON = 1e-6f;
const float EPSILON_COMPARE = 1e-12f;

float delta_distance = 0.0;  
float distance = 0.0;

/*==== Fungsi bantuan untuk clamp ====*/
inline float clamp(float x, float lower, float upper) {
    return fmaxf(lower, fminf(x, upper));
}

/*==== Util MF yang robust ====*/
inline float trimf(float x, float a, float b, float c) {
    // Triangular: a < b < c
    if (x <= a || x >= c) return 0.0f;
    if (fabsf(x - b) < EPSILON_COMPARE) return 1.0f;
    if (x < b)  return (x - a) / fmaxf(b - a, EPSILON);
    return (c - x) / fmaxf(c - b, EPSILON);
}

inline float trapmf(float x, float a, float b, float c, float d) {
    // Trapezoid: a <= b <= c <= d, handle shoulders when a==b or c==d
    if (x < a) return 0.0f;
    else if (x < b) return (x - a) / fmaxf(b - a, EPSILON);
    else if (x <= c) return 1.0f;                     // plateau [b, c]
    else if (x < d)  return (d - x) / fmaxf(d - c, EPSILON);
    else return 0.0f;
}

/*==== Membership function Delta Distance (ingat: negatif = mendekat) ====*/
// 'Mendekat' (shoulder kiri): plateau dari -1.0 s/d -0.5, turun ke 0 di -0.1
float delta_distance_membership_mendekat(float x) {
    return trapmf(x, -1.0f, -1.0f, -0.5f, -0.1f);
}

// 'Stabil' (segitiga di sekitar 0)
float delta_distance_membership_stabil(float x) {
    return trimf(x, -0.2f, 0.0f, 0.2f);
}

// 'Menjauh' (shoulder kanan): naik dari 0.1 ke 0.5, plateau s/d 1.0
float delta_distance_membership_menjauh(float x) {
    return trapmf(x,  0.1f,  0.5f, 1.0f, 1.0f);
}

/*==== Membership function Delta Distance END ====*/

/*==== Membership function Distance START ====*/
float distance_membership_intimate(float x) {
    if (x < 0.0 || x > 0.5) {
        return 0.0;
    }
    return (0.5 - x) / 0.5;
}

float distance_membership_personal(float x) {
    if (x < 0.0 || x > 1.2) {
        return 0.0;
    }
    if (x <= 0.5) {
        return x / 0.5;
    } else {
        return (1.2 - x) / (1.2 - 0.5); 
    }
}

float distance_membership_social(float x) {
    if (x < 0.5 || x > 3.5) {
        return 0.0;
    }
    if (x <= 1.2) {
        return (x - 0.5) / (1.2 - 0.5);
    } else {
        return (3.5 - x) / (3.5 - 1.2);
    }
}

float distance_membership_public(float x) {
    if (x <= 1.2) {
        return 0.0;
    }
    if (x >= 3.5) {
        return 1.0;
    }
    return (x - 1.2) / (3.5 - 1.2);
}
/*==== Membership function Distance END ====*/

/*==== Membership function output (0 to 1) START ====*/
float calculate_fuzzy_output(float current_distance, float current_delta_distance) {
    float delta_for_fuzzy = clamp(current_delta_distance, -1.0f, 1.0f);
    // === Langkah 1: Fuzzifikasi ===
    float mu_delta_mendekat = delta_distance_membership_mendekat(delta_for_fuzzy);
    float mu_delta_stabil = delta_distance_membership_stabil(delta_for_fuzzy);
    float mu_delta_menjauh = delta_distance_membership_menjauh(delta_for_fuzzy);

    float mu_dist_intimate = distance_membership_intimate(current_distance);
    float mu_dist_personal = distance_membership_personal(current_distance);
    float mu_dist_social = distance_membership_social(current_distance);
    float mu_dist_public = distance_membership_public(current_distance);

    // === Langkah 2: Evaluasi Aturan (Inference) ===
    // Aturan untuk Delta Distance = Menjauh (Aman, robot menjauhi obstacle)
    float w1 = fminf(mu_delta_menjauh, mu_dist_intimate);   
    float w2 = fminf(mu_delta_menjauh, mu_dist_personal);  
    float w3 = fminf(mu_delta_menjauh, mu_dist_social);   
    float w4 = fminf(mu_delta_menjauh, mu_dist_public);   

    // Aturan untuk Delta Distance = Stabil
    float w5 = fminf(mu_delta_stabil, mu_dist_intimate);  
    float w6 = fminf(mu_delta_stabil, mu_dist_personal);
    float w7 = fminf(mu_delta_stabil, mu_dist_social);  
    float w8 = fminf(mu_delta_stabil, mu_dist_public); 

    // Aturan untuk Delta Distance = Mendekat (Bahaya, obstacle mendekat)
    float w9 = fminf(mu_delta_mendekat, mu_dist_intimate);  
    float w10 = fminf(mu_delta_mendekat, mu_dist_personal); 
    float w11 = fminf(mu_delta_mendekat, mu_dist_social);   
    float w12 = fminf(mu_delta_mendekat, mu_dist_public);   

    // === Langkah 3: Defuzzifikasi (Weighted Average) ===
    // Menghitung keluaran crisp. Rumus: Σ(wi * zi) / Σ(wi)

    //PROXEMIC STATIC
    // float numerator = (w1 * 0.495835) + (w2 * 0.036861) + (w3 * 0.907595) + (w4 * 0.513573) +
    //                 (w5 * 0.458004) + (w6 * 0.507346) + (w7 * 0.452850) + (w8 * 0.815666) +
    //                 (w9 * 0.562869) + (w10 * 0.542954) + (w11 * 0.448947) + (w12 * 0.490455);

    // float denominator = w1 + w2 + w3 + w4 + w5 + w6 + w7 + w8 + w9 + w10 + w11 + w12;

    //PROXEMIC DINAMIS
     float numerator = (w1 * 0.630446) + (w2 * 0.605582) + (w3 * 0.562108) + (w4 * 0.680821) +
                        (w5 * 0.486959) + (w6 * 0.656589) + (w7 * 0.630166) + (w8 * 0.887996) +
                        (w9 * 0.631297) + (w10 * 0.503137) + (w11 * 0.481471) + (w12 * 0.674134);

     float denominator = w1 + w2 + w3 + w4 + w5 + w6 + w7 + w8 + w9 + w10 + w11 + w12;

    //DEFAULT PROPOSAL
    // float numerator = (w1 * 0.6) + (w2 * 0.4) + (w3 * 0.2) + (w4 * 0.1) +
    //                   (w5 * 0.8) + (w6 * 0.6) + (w7 * 0.4) + (w8 * 0.2) +
    //                   (w9 * 1.0) + (w10 * 0.8) + (w11 * 0.5) + (w12 * 0.3);

    // float denominator = w1 + w2 + w3 + w4 + w5 + w6 + w7 + w8 + w9 + w10 + w11 + w12;

    // Menghindari pembagian dengan nol jika tidak ada aturan yang aktif
    if (denominator == 0) {
        return 0.0;
    }

    return numerator / denominator;
}

void distanceDeltaCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    if (msg->data.size() < 2) {
        ROS_WARN("Pesan /near_distance tidak lengkap.");
        return;
    }
    distance = msg->data[0];
    delta_distance = msg->data[1];
    ROS_DEBUG("Fuzzy received distance: %.2f, delta_distance: %.2f", distance, delta_distance);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fuzzy_controller");
    ros::NodeHandle nh;

    // Publisher - Output to SFM node
    ros::Publisher fuzzy_pub = nh.advertise<std_msgs::Float32>("/fuzzy", 10);

    // Subscriber - Input from SFM node (distance and delta_distance)
    ros::Subscriber sub_distance = nh.subscribe("/near_distance", 10, distanceDeltaCallback);
    
    ros::Rate rate(30); // 30 Hz sama seperti package ackerman
    
    while (ros::ok()) {
        float acceleration = calculate_fuzzy_output(distance, delta_distance);

        ROS_DEBUG("Distance: %.2f, Delta Distance: %.2f, Fuzzy Gain: %.2f", distance, delta_distance, acceleration);
        
        // Publish fuzzy gain to SFM node
        static std_msgs::Float32 gain;
        gain.data = acceleration; 
        fuzzy_pub.publish(gain);

        ros::spinOnce(); 
        rate.sleep();
    }

    return 0;
}
