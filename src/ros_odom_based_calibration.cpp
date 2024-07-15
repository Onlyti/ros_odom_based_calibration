/**
 * @file ros_odom_based_calibration.cpp
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

#include "ros_odom_based_calibration.hpp"

RosOdomBasedCalibration::RosOdomBasedCalibration()
    : deq_odom1_delta_(),
      deq_odom2_delta_(),
      deq_odom_pair_(),
      odom1_topic_(""),
      odom2_topic_(""),
      odom1_type_(OdomType::ODOM_TYPE_NAV_MSGS_ODOM),
      odom2_type_(OdomType::ODOM_TYPE_NAV_MSGS_ODOM),
      str_ini_path_(""),
      cfg_i_que_max_size_(0),
      cfg_d_keyframe_dist_threshold_m_(0.0),
      cfg_d_keyframe_rot_threshold_deg_(0.0),
      cfg_d_time_delay_odom1_to_odom2_sec_(0.0),
      cfg_d_cal_t_x_m_(0.0),
      cfg_d_cal_t_y_m_(0.0),
      cfg_d_cal_t_z_m_(0.0),
      cfg_d_fde_max_translation_error_threshold_m_(0.0),
      cfg_d_fde_max_rotation_error_threshold_deg_(0.0) {
    // Read ROS parameters
    bool ros_param_read_state = RosParamRead();
    if (ros_param_read_state == false) {
        DebugPrintError("[RosOdomBasedCalibration] Failed to read ROS parameters");
        return;
    }

    // Initialize ini handler
    bool ini_read_state = ini_handler_.Init(str_ini_path_);
    if (ini_read_state == false) {
        DebugPrintError("[RosOdomBasedCalibration] Failed to read INI file");
        return;
    }
    // Parse INI file
    bool ini_parse_state = ParseINI();
    if (ini_parse_state == false) {
        DebugPrintError("[RosOdomBasedCalibration] Failed to parse INI file");
        return;
    }

    // ROS
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // Subscribers
    if (odom1_type_ == OdomType::ODOM_TYPE_NAV_MSGS_ODOM) {
        rossub_odom1_ = nh.subscribe(odom1_topic_, 1, &RosOdomBasedCalibration::CallbackOdom1Odom, this);
    }
    else if (odom1_type_ == OdomType::ODOM_TYPE_GEO_MSGS_TWIST) {
        rossub_odom1_ = nh.subscribe(odom1_topic_, 1, &RosOdomBasedCalibration::CallbackOdom1Twist, this);
    }
    else if (odom1_type_ == OdomType::ODOM_TYPE_GEO_MSGS_POSESTAMPED) {
        rossub_odom1_ = nh.subscribe(odom1_topic_, 1, &RosOdomBasedCalibration::CallbackOdom1Pose, this);
    }

    if (odom2_type_ == OdomType::ODOM_TYPE_NAV_MSGS_ODOM) {
        rossub_odom2_ = nh.subscribe(odom2_topic_, 1, &RosOdomBasedCalibration::CallbackOdom2Odom, this);
    }
    else if (odom2_type_ == OdomType::ODOM_TYPE_GEO_MSGS_TWIST) {
        rossub_odom2_ = nh.subscribe(odom2_topic_, 1, &RosOdomBasedCalibration::CallbackOdom2Twist, this);
    }
    else if (odom2_type_ == OdomType::ODOM_TYPE_GEO_MSGS_POSESTAMPED) {
        rossub_odom2_ = nh.subscribe(odom2_topic_, 1, &RosOdomBasedCalibration::CallbackOdom2Pose, this);
    }

    // Publishers
    rospub_calib_odom1_to_odom2_ = nh.advertise<geometry_msgs::PoseStamped>("calib_odom1_to_odom2", 1);
    rospub_fde_translation_error_m_ = nh.advertise<std_msgs::Float64>("fde_translation_error_m", 1);
    rospub_fde_rotation_error_deg_ = nh.advertise<std_msgs::Float64>("fde_rotation_error_deg", 1);
}
RosOdomBasedCalibration::~RosOdomBasedCalibration() {}

// Initialization
/**
 * @brief Read ROS parameters
 *
 * @return true
 * @return false
 */
bool RosOdomBasedCalibration::RosParamRead() {
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // Topic names
    nh_private.param<std::string>("odom1_topic", odom1_topic_, "");
    if (odom1_topic_.empty()) {
        DebugPrintError("[RosOdomBasedCalibration] Odom1 topic is empty");
        return false;
    }
    else {
        DebugPrintInfo("[RosOdomBasedCalibration] Odom1 topic: " + odom1_topic_);
    }
    nh_private.param<std::string>("odom2_topic", odom2_topic_, "");
    if (odom2_topic_.empty()) {
        DebugPrintError("[RosOdomBasedCalibration] Odom2 topic is empty");
        return false;
    }
    else {
        DebugPrintInfo("[RosOdomBasedCalibration] Odom2 topic: " + odom2_topic_);
    }

    // Topic types
    std::string tmp_str = "";
    nh_private.param<std::string>("odom1_type", tmp_str, "");
    if (tmp_str == "ODOM_TYPE_NAV_MSGS_ODOM")
        odom1_type_ = OdomType::ODOM_TYPE_NAV_MSGS_ODOM;
    else if (tmp_str == "ODOM_TYPE_GEO_MSGS_TWIST")
        odom1_type_ = OdomType::ODOM_TYPE_GEO_MSGS_TWIST;
    else if (tmp_str == "ODOM_TYPE_GEO_MSGS_POSESTAMPED")
        odom1_type_ = OdomType::ODOM_TYPE_GEO_MSGS_POSESTAMPED;
    else {
        DebugPrintError("[RosOdomBasedCalibration] Odom1 type is invalid");
        return false;
    }

    tmp_str = "";
    nh_private.param<std::string>("odom2_type", tmp_str, "");
    if (tmp_str == "ODOM_TYPE_NAV_MSGS_ODOM")
        odom2_type_ = OdomType::ODOM_TYPE_NAV_MSGS_ODOM;
    else if (tmp_str == "ODOM_TYPE_GEO_MSGS_TWIST")
        odom2_type_ = OdomType::ODOM_TYPE_GEO_MSGS_TWIST;
    else if (tmp_str == "ODOM_TYPE_GEO_MSGS_POSESTAMPED")
        odom2_type_ = OdomType::ODOM_TYPE_GEO_MSGS_POSESTAMPED;
    else {
        DebugPrintError("[RosOdomBasedCalibration] Odom2 type is invalid");
        return false;
    }

    nh_private.param<std::string>("ini_path", str_ini_path_, "");
    if (str_ini_path_.empty()) {
        DebugPrintError("[RosOdomBasedCalibration] INI file path is empty");
        return false;
    }
    else {
        DebugPrintInfo("[RosOdomBasedCalibration] INI file path: " + str_ini_path_);
    }
}
/**
 * @brief Parse INI file
 *
 * @return true
 * @return false
 */
bool RosOdomBasedCalibration::ParseINI() {
    if (ini_handler_.IsFileUpdated() == true) {
        // Que and keyframe parameters
        if (ini_handler_.ParseConfig("Config", "cfg_i_que_max_size_", cfg_i_que_max_size_) == false) {
            DebugPrintError("[RosOdomBasedCalibration] Failed to parse cfg_i_que_max_size_");
            return false;
        }
        if (cfg_i_que_max_size_ <= 0) {
            DebugPrintError("[RosOdomBasedCalibration] cfg_i_que_max_size_ should be positive value");
            return false;
        }
        if (ini_handler_.ParseConfig("Config", "cfg_d_keyframe_dist_threshold_m_", cfg_d_keyframe_dist_threshold_m_) ==
            false) {
            DebugPrintError("[RosOdomBasedCalibration] Failed to parse cfg_d_keyframe_dist_threshold_m_");
            return false;
        }
        if (cfg_d_keyframe_dist_threshold_m_ < 0.0) {
            DebugPrintError("[RosOdomBasedCalibration] cfg_d_keyframe_dist_threshold_m_ should be positive value");
            return false;
        }
        if (ini_handler_.ParseConfig("Config", "cfg_d_keyframe_rot_threshold_deg_",
                                     cfg_d_keyframe_rot_threshold_deg_) == false) {
            DebugPrintError("[RosOdomBasedCalibration] Failed to parse cfg_d_keyframe_rot_threshold_deg_");
            return false;
        }
        if (cfg_d_keyframe_rot_threshold_deg_ < 0.0) {
            DebugPrintError("[RosOdomBasedCalibration] cfg_d_keyframe_rot_threshold_deg_ should be positive value");
            return false;
        }
        if (ini_handler_.ParseConfig("Config", "cfg_d_time_delay_odom1_to_odom2_sec_",
                                     cfg_d_time_delay_odom1_to_odom2_sec_) == false) {
            DebugPrintError("[RosOdomBasedCalibration] Failed to parse cfg_d_time_delay_odom1_to_odom2_sec_");
            return false;
        }
        if (cfg_d_time_delay_odom1_to_odom2_sec_ < 0.0) {
            DebugPrintError("[RosOdomBasedCalibration] cfg_d_time_delay_odom1_to_odom2_sec_ should be positive value");
            return false;
        }

        // Predefined translation calibration parameters
        if (ini_handler_.ParseConfig("Cal", "cfg_d_cal_t_x_m_", cfg_d_cal_t_x_m_) == false) {
            DebugPrintError("[RosOdomBasedCalibration] Failed to parse cfg_d_cal_t_x_m_");
            return false;
        }
        if (ini_handler_.ParseConfig("Cal", "cfg_d_cal_t_y_m_", cfg_d_cal_t_y_m_) == false) {
            DebugPrintError("[RosOdomBasedCalibration] Failed to parse cfg_d_cal_t_y_m_");
            return false;
        }
        if (ini_handler_.ParseConfig("Cal", "cfg_d_cal_t_z_m_", cfg_d_cal_t_z_m_) == false) {
            DebugPrintError("[RosOdomBasedCalibration] Failed to parse cfg_d_cal_t_z_m_");
            return false;
        }

        // Fault Data Exclusion
        if (ini_handler_.ParseConfig("FDE", "cfg_d_fde_max_translation_error_threshold_m_",
                                     cfg_d_fde_max_translation_error_threshold_m_) == false) {
            DebugPrintError("[RosOdomBasedCalibration] Failed to parse cfg_d_fde_max_translation_error_threshold_m_");
            return false;
        }
        if (ini_handler_.ParseConfig("FDE", "cfg_d_fde_max_rotation_error_threshold_deg_",
                                     cfg_d_fde_max_rotation_error_threshold_deg_) == false) {
            DebugPrintError("[RosOdomBasedCalibration] Failed to parse cfg_d_fde_max_rotation_error_threshold_deg_");
            return false;
        }
    }
    return true;
}

// Algorithm
void RosOdomBasedCalibration::Run() {
    ParseINI();
    SyncOdomsTimeSync();
    FaultDataExclusion();
    IterativeRotationEstimation();
}

void RosOdomBasedCalibration::SyncOdomsTimeSync() {
    // Synchronize odometry data
    if (deq_odom1_delta_.empty() || deq_odom2_delta_.empty()) {
        DebugPrintWarn("[RosOdomBasedCalibration] Odom1 or Odom2 delta is empty");
        return;
    }

    // Time synchronization


}

void RosOdomBasedCalibration::FaultDataExclusion() {}

void RosOdomBasedCalibration::IterativeRotationEstimation() {}

// ROS Callbacks
void RosOdomBasedCalibration::CallbackOdom1Odom(const nav_msgs::Odometry::ConstPtr &msg) {
    // Convert nav_msgs::Odometry (global frame odometry) to Eigen::Affine3d (difference odometry)
    static nav_msgs::Odometry odom1_prev = *msg;
    nav_msgs::Odometry odom1_curr = *msg;

    // Determine delta odometry
    Eigen::Affine3d odom1_delta;
    odom1_delta.setIdentity();
    // Translation
    odom1_delta.translation() = Eigen::Vector3d(odom1_curr.pose.pose.position.x - odom1_prev.pose.pose.position.x,
                                                odom1_curr.pose.pose.position.y - odom1_prev.pose.pose.position.y,
                                                odom1_curr.pose.pose.position.z - odom1_prev.pose.pose.position.z);

    // Rotation
    Eigen::Quaterniond q_g_to_1_prev(odom1_prev.pose.pose.orientation.w, odom1_prev.pose.pose.orientation.x,
                                     odom1_prev.pose.pose.orientation.y, odom1_prev.pose.pose.orientation.z);
    Eigen::Quaterniond q_g_to_1_curr(odom1_curr.pose.pose.orientation.w, odom1_curr.pose.pose.orientation.x,
                                     odom1_curr.pose.pose.orientation.y, odom1_curr.pose.pose.orientation.z);
    Eigen::Quaterniond q_prev_to_curr = q_g_to_1_prev.inverse() * q_g_to_1_curr;
    Eigen::AngleAxisd aa_prev_to_curr(q_prev_to_curr);
    odom1_delta.rotate(aa_prev_to_curr);

    // Push back
    double time_sec = odom1_curr.header.stamp.toSec();
    deq_odom1_delta_.push_back(std::make_pair(time_sec, odom1_delta));
    QueLimiters(deq_odom1_delta_);
    odom1_prev = odom1_curr;
    return;
}

void RosOdomBasedCalibration::CallbackOdom1Twist(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    // Convert geometry_msgs::TwistStamped (global frame odometry) to Eigen::Affine3d (difference odometry)
    static geometry_msgs::TwistStamped odom1_prev = *msg;
    geometry_msgs::TwistStamped odom1_curr = *msg;

    // Determine delta odometry
    Eigen::Affine3d odom1_delta;
    odom1_delta.setIdentity();
    double delta_time_sec = time_sec - odom1_prev.header.stamp.toSec();
    // Translation
    double dx = (odom1_curr.twist.linear.x - odom1_prev.twist.linear.x) * delta_time_sec;
    double dy = (odom1_curr.twist.linear.y - odom1_prev.twist.linear.y) * delta_time_sec;
    double dz = (odom1_curr.twist.linear.z - odom1_prev.twist.linear.z) * delta_time_sec;
    odom1_delta.translation() = Eigen::Vector3d(dx, dy, dz);

    // Rotation
    double droll = (odom1_curr.twist.angular.x - odom1_prev.twist.angular.x) * delta_time_sec;
    double dpitch = (odom1_curr.twist.angular.y - odom1_prev.twist.angular.y) * delta_time_sec;
    double dyaw = (odom1_curr.twist.angular.z - odom1_prev.twist.angular.z) * delta_time_sec;
    // Convert Euler angle to quaternion
    Eigen::Quaterniond q_prev_to_curr = Eigen::AngleAxisd(droll, Eigen::Vector3d::UnitX()) *
                                        Eigen::AngleAxisd(dpitch, Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd aa_prev_to_curr(q_prev_to_curr);
    odom1_delta.rotate(aa_prev_to_curr);

    // Push back
    double time_sec = odom1_curr.header.stamp.toSec();
    deq_odom1_delta_.push_back(std::make_pair(time_sec, odom1_delta));
    QueLimiters(deq_odom1_delta_);
    odom1_prev = odom1_curr;
    return;
}

void RosOdomBasedCalibration::CallbackOdom1Pose(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // Convert geometry_msgs::PoseStamped (global frame odometry) to Eigen::Affine3d (difference odometry)
    static geometry_msgs::PoseStamped odom1_prev = *msg;
    geometry_msgs::PoseStamped odom1_curr = *msg;

    // Determine delta odometry
    Eigen::Affine3d odom1_delta;
    odom1_delta.setIdentity();
    // Translation
    odom1_delta.translation() = Eigen::Vector3d(odom1_curr.pose.position.x - odom1_prev.pose.position.x,
                                                odom1_curr.pose.position.y - odom1_prev.pose.position.y,
                                                odom1_curr.pose.position.z - odom1_prev.pose.position.z);

    // Rotation
    Eigen::Quaterniond q_g_to_1_prev(odom1_prev.pose.orientation.w, odom1_prev.pose.orientation.x,
                                     odom1_prev.pose.orientation.y, odom1_prev.pose.orientation.z);
    Eigen::Quaterniond q_g_to_1_curr(odom1_curr.pose.orientation.w, odom1_curr.pose.orientation.x,
                                     odom1_curr.pose.orientation.y, odom1_curr.pose.orientation.z);
    Eigen::Quaterniond q_prev_to_curr = q_g_to_1_prev.inverse() * q_g_to_1_curr;
    Eigen::AngleAxisd aa_prev_to_curr(q_prev_to_curr);
    odom1_delta.rotate(aa_prev_to_curr);

    // Push back
    double time_sec = odom1_curr.header.stamp.toSec();
    deq_odom1_delta_.push_back(std::make_pair(time_sec, odom1_delta));
    QueLimiters(deq_odom1_delta_);
    odom1_prev = odom1_curr;
    return;
}

void RosOdomBasedCalibration::CallbackOdom2Odom(const nav_msgs::Odometry::ConstPtr &msg) {
    // Convert nav_msgs::Odometry (global frame odometry) to Eigen::Affine3d (difference odometry)
    static nav_msgs::Odometry odom2_prev = *msg;
    nav_msgs::Odometry odom2_curr = *msg;

    // Determine delta odometry
    Eigen::Affine3d odom2_delta;
    odom2_delta.setIdentity();
    // Translation
    odom2_delta.translation() = Eigen::Vector3d(odom2_curr.pose.pose.position.x - odom2_prev.pose.pose.position.x,
                                                odom2_curr.pose.pose.position.y - odom2_prev.pose.pose.position.y,
                                                odom2_curr.pose.pose.position.z - odom2_prev.pose.pose.position.z);

    // Rotation
    Eigen::Quaterniond q_g_to_2_prev(odom2_prev.pose.pose.orientation.w, odom2_prev.pose.pose.orientation.x,
                                     odom2_prev.pose.pose.orientation.y, odom2_prev.pose.pose.orientation.z);
    Eigen::Quaterniond q_g_to_2_curr(odom2_curr.pose.pose.orientation.w, odom2_curr.pose.pose.orientation.x,
                                     odom2_curr.pose.pose.orientation.y, odom2_curr.pose.pose.orientation.z);
    Eigen::Quaterniond q_prev_to_curr = q_g_to_2_prev.inverse() * q_g_to_2_curr;
    Eigen::AngleAxisd aa_prev_to_curr(q_prev_to_curr);
    odom2_delta.rotate(aa_prev_to_curr);

    // Push back
    double time_sec = odom2_curr.header.stamp.toSec() - cfg_d_time_delay_odom1_to_odom2_sec_;
    deq_odom2_delta_.push_back(std::make_pair(time_sec, odom2_delta));
    QueLimiters(deq_odom2_delta_);
    odom2_prev = odom2_curr;
    return;
}

void RosOdomBasedCalibration::CallbackOdom2Twist(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    // Convert geometry_msgs::TwistStamped (global frame odometry) to Eigen::Affine3d (difference odometry)
    static geometry_msgs::TwistStamped odom2_prev = *msg;
    geometry_msgs::TwistStamped odom2_curr = *msg;

    // Determine delta odometry
    Eigen::Affine3d odom2_delta;
    odom2_delta.setIdentity();
    double delta_time_sec = time_sec - odom2_prev.header.stamp.toSec();
    // Translation
    double dx = (odom2_curr.twist.linear.x - odom2_prev.twist.linear.x) * delta_time_sec;
    double dy = (odom2_curr.twist.linear.y - odom2_prev.twist.linear.y) * delta_time_sec;
    double dz = (odom2_curr.twist.linear.z - odom2_prev.twist.linear.z) * delta_time_sec;
    odom2_delta.translation() = Eigen::Vector3d(dx, dy, dz);

    // Rotation
    double droll = (odom2_curr.twist.angular.x - odom2_prev.twist.angular.x) * delta_time_sec;
    double dpitch = (odom2_curr.twist.angular.y - odom2_prev.twist.angular.y) * delta_time_sec;
    double dyaw = (odom2_curr.twist.angular.z - odom2_prev.twist.angular.z) * delta_time_sec;
    // Convert Euler angle to quaternion
    Eigen::Quaterniond q_prev_to_curr = Eigen::AngleAxisd(droll, Eigen::Vector3d::UnitX()) *
                                        Eigen::AngleAxisd(dpitch, Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(dyaw, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd aa_prev_to_curr(q_prev_to_curr);
    odom2_delta.rotate(aa_prev_to_curr);

    // Push back
    double time_sec = odom2_curr.header.stamp.toSec() - cfg_d_time_delay_odom1_to_odom2_sec_;
    deq_odom2_delta_.push_back(std::make_pair(time_sec, odom2_delta));
    QueLimiters(deq_odom2_delta_);
    odom2_prev = odom2_curr;
    return;
}

void RosOdomBasedCalibration::CallbackOdom2Pose(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // Convert geometry_msgs::PoseStamped (global frame odometry) to Eigen::Affine3d (difference odometry)
    static geometry_msgs::PoseStamped odom2_prev = *msg;
    geometry_msgs::PoseStamped odom2_curr = *msg;

    // Determine delta odometry
    Eigen::Affine3d odom2_delta;
    odom2_delta.setIdentity();
    // Translation
    odom2_delta.translation() = Eigen::Vector3d(odom2_curr.pose.position.x - odom2_prev.pose.position.x,
                                                odom2_curr.pose.position.y - odom2_prev.pose.position.y,
                                                odom2_curr.pose.position.z - odom2_prev.pose.position.z);

    // Rotation
    Eigen::Quaterniond q_g_to_2_prev(odom2_prev.pose.orientation.w, odom2_prev.pose.orientation.x,
                                     odom2_prev.pose.orientation.y, odom2_prev.pose.orientation.z);
    Eigen::Quaterniond q_g_to_2_curr(odom2_curr.pose.orientation.w, odom2_curr.pose.orientation.x,
                                     odom2_curr.pose.orientation.y, odom2_curr.pose.orientation.z);
    Eigen::Quaterniond q_prev_to_curr = q_g_to_2_prev.inverse() * q_g_to_2_curr;
    Eigen::AngleAxisd aa_prev_to_curr(q_prev_to_curr);
    odom2_delta.rotate(aa_prev_to_curr);

    // Push back
    double time_sec = odom2_curr.header.stamp.toSec() - cfg_d_time_delay_odom1_to_odom2_sec_;
    deq_odom2_delta_.push_back(std::make_pair(time_sec, odom2_delta));
    QueLimiters(deq_odom2_delta_);
    odom2_prev = odom2_curr;
    return;
}

void RosOdomBasedCalibration::QueLimiters(std::deque<std::pair<double, Eigen::Affine3d>> &deq_odom_delta_){
    // Limit the size of the queue
    while (deq_odom_delta_.size() > cfg_i_que_max_size_) {
        deq_odom_delta_.pop_front();
    }
}

// Main
int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_odom_based_calibration");
    RosOdomBasedCalibration ros_odom_based_calibration;

    double frequency = 1.0;
    ros::Rate loop_rate(frequency);
    while (ros::ok()) {
        ros_odom_based_calibration.Run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}