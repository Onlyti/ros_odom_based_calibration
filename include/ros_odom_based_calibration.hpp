/**
 * @file ros_odom_based_calibration.hpp
 * @author Jiwon Seok (pauljiwon96@gmail.com)
 * @brief Odometry based rotation calibration with Fault Data Exclusion, Alborithm is based on extension of the paper
"Fault Detection and Exclusion for Robust Online Calibration of Vehicle to LiDAR Rotation Parameter" by Jiwon Seok,
Chansoo Kim, Paulo Resende, Benazouz Bradai, and Kichun Jo
 * @version 0.1
 * @date 2024-07-15
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef __ROS_ODOM_BASED_CALIBRATION__
#define __ROS_ODOM_BASED_CALIBRATION__

// Headers
#include <algorithm>
#include <cmath>
#include <deque>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <ros/ros.h>

// Utility
#include "debugprintutil/debug_print.hpp"
#include "ini_handler_cpp/c_ini.hpp"

// Defines & types
enum OdomType { ODOM_TYPE_NAV_MSGS_ODOM = 0, ODOM_TYPE_GEO_MSGS_TWIST, ODOM_TYPE_GEO_MSGS_POSESTAMPED };

class RosOdomBasedCalibration {
public:
    RosOdomBasedCalibration();
    ~RosOdomBasedCalibration();

    // Initialization
    bool RosParamRead();
    bool ParseINI();

    // Algorithm
    void Run();
    void SyncOdomsTimeSync();
    void FaultDataExclusion();
    void IterativeRotationEstimation();

    // ROS Callbacks
    void CallbackOdom1Odom(const nav_msgs::Odometry::ConstPtr &msg);
    void CallbackOdom1Twist(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void CallbackOdom1Pose(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void CallbackOdom2Odom(const nav_msgs::Odometry::ConstPtr &msg);
    void CallbackOdom2Twist(const geometry_msgs::TwistStamped::ConstPtr &msg);
    void CallbackOdom2Pose(const geometry_msgs::PoseStamped::ConstPtr &msg);

    void QueLimiters(std::deque<std::pair<double, Eigen::Affine3d>>& deq_odom_delta_);

public:
    ros::Subscriber rossub_odom1_;
    ros::Subscriber rossub_odom2_;

    ros::Publisher rospub_calib_odom1_to_odom2_;
    ros::Publisher rospub_fde_translation_error_m_;
    ros::Publisher rospub_fde_rotation_error_deg_;


    // Inputs
    std::deque<std::pair<double, Eigen::Affine3d>>
            deq_odom1_delta_; // Robust odometry like vehicle odometry, assume as reference frame
    std::deque<std::pair<double, Eigen::Affine3d>> deq_odom2_delta_; // Target odometry frame

    std::deque<std::pair<Eigen::Affine3d, Eigen::Affine3d>> deq_odom_pair_;

    // Estimated rotation
    double d_estimated_roll_rad_;
    double d_estimated_pitch_rad_;
    double d_estimated_yaw_rad_;

    // Configurations
    std::string odom1_topic_;
    std::string odom2_topic_;
    OdomType odom1_type_;
    OdomType odom2_type_;

    // Parameters
    CINI_H ini_handler_;
    std::string str_ini_path_;
    // Que and keyframe parameters
    int cfg_i_que_max_size_;                     // Sliding window for calibration
    double cfg_d_keyframe_dist_threshold_m_;     // Keyframe distance threshold
    double cfg_d_keyframe_rot_threshold_deg_;    // Keyframe rotation threshold
    double cfg_d_time_delay_odom1_to_odom2_sec_; // time_delay = odom2_time - odom1_time,
                                                 // odom1_time = odom2_time - time_delay

    // Predefined translation calibration parameters
    double cfg_d_cal_t_x_m_;
    double cfg_d_cal_t_y_m_;
    double cfg_d_cal_t_z_m_;

    // Fault Data Exclusion
    double cfg_d_fde_max_translation_error_threshold_m_;
    double cfg_d_fde_max_rotation_error_threshold_deg_;
};

#endif // __ROS_ODOM_BASED_CALIBRATION__