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
    : deq_odom1_accumed_(),
      deq_odom2_accumed_(),
      deq_odom_pair_(),
      odom1_topic_(""),
      odom2_topic_(""),
      odom1_type_(OdomType::ODOM_TYPE_NAV_MSGS_ODOM),
      odom2_type_(OdomType::ODOM_TYPE_NAV_MSGS_ODOM),
      str_ini_path_(""),
      cfg_i_que_input_max_size_(0),
      cfg_i_que_keyframe_max_size_(0),
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
        if (ini_handler_.ParseConfig("Config", "cfg_i_que_input_max_size_", cfg_i_que_input_max_size_) == false) {
            DebugPrintError("[RosOdomBasedCalibration] Failed to parse cfg_i_que_input_max_size_");
            return false;
        }
        if (cfg_i_que_input_max_size_ <= 0) {
            DebugPrintError("[RosOdomBasedCalibration] cfg_i_que_input_max_size_ should be positive value");
            return false;
        }
        if (ini_handler_.ParseConfig("Config", "cfg_i_que_keyframe_max_size_", cfg_i_que_keyframe_max_size_) == false) {
            DebugPrintError("[RosOdomBasedCalibration] Failed to parse cfg_i_que_keyframe_max_size_");
            return false;
        }
        if (cfg_i_que_keyframe_max_size_ <= 0) {
            DebugPrintError("[RosOdomBasedCalibration] cfg_i_que_keyframe_max_size_ should be positive value");
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
    // Check if the queue is empty
    if (deq_odom1_accumed_.size() < 2 || deq_odom2_accumed_.size() < 2) {
        DebugPrintWarn("[RosOdomBasedCalibration] Odom1 or Odom2 delta is empty");
        return;
    }

    // Check start condition: odom1 front time > odom2 front time and odom1 front time < odom2 back time
    static bool b_start_condition = false;
    if (b_start_condition == false) {
        while (deq_odom1_accumed_.size() > 1 && deq_odom2_accumed_.size() > 1) {
            double odom1_front_time_sec = deq_odom1_accumed_.front().time_sec;
            double odom2_front_time_sec = deq_odom2_accumed_.front().time_sec;
            double odom2_back_time_sec = deq_odom2_accumed_.back().time_sec;

            if (odom1_front_time_sec > odom2_front_time_sec && odom1_front_time_sec < odom2_back_time_sec) {
                double odom2_second_time_sec = deq_odom2_accumed_.at(1).time_sec;

                // Check if the time of odom1 is in between the time of odom2
                while (odom1_front_time_sec > odom2_second_time_sec) {
                    deq_odom2_accumed_.pop_front();

                    if (deq_odom2_accumed_.size() < 2) {
                        DebugPrintWarn("[RosOdomBasedCalibration] Odom2 time is not synchronized");
                        return;
                    }

                    odom2_front_time_sec = deq_odom2_accumed_.front().time_sec;
                    odom2_second_time_sec = deq_odom2_accumed_.at(1).time_sec;
                }

                b_start_condition = true;
                last_keyframe_time_sec_ = odom1_front_time_sec;
                last_keyframe_odom1_ = deq_odom1_accumed_.front().odom;

                // Synchronize odom2 time from odom1 time with interpolation
                double ratio =
                        (odom1_front_time_sec - odom2_front_time_sec) / (odom2_second_time_sec - odom2_front_time_sec);
                last_keyframe_odom2_ =
                        GetIntpAffine3d(deq_odom2_accumed_.front().odom, deq_odom2_accumed_.at(1).odom, ratio);

                break; // Exit the loop once the start condition is met
            }
            else {
                DebugPrintWarn("[RosOdomBasedCalibration] Odom1 and Odom2 time is in synchronization progress");
                deq_odom1_accumed_.pop_front();
                return;
            }
        }
    }

    /* Keyframe Selection */
    // Keyframe generation condition
    // Find the odom1 last time smaller then odom2 last time (for interpolation)
    double odom1_front_time = deq_odom1_accumed_.front().time_sec;
    double odom1_last_time_sec = odom1_front_time;
    int odom1_last_idx = 0;
    for (int odom1_idx = deq_odom1_accumed_.size() - 1; odom1_idx >= 0; odom1_idx--) {
        if (deq_odom1_accumed_.at(odom1_idx).time_sec < deq_odom2_accumed_.back().time_sec) {
            odom1_last_time_sec = deq_odom1_accumed_.at(odom1_idx).time_sec;
            odom1_last_idx = odom1_idx;
            break;
        }
    }
    // Check if the odom1 last time is found
    if (odom1_last_time_sec <= odom1_front_time) {
        DebugPrintWarn("[RosOdomBasedCalibration] Odom1 last time is not found");
        return;
    }

    // Check difference between the last keyframe and the last odom1
    Eigen::Affine3d odom1_delta_fron_last_kf = last_keyframe_odom1_.inverse() * deq_odom1_accumed_.back().odom;
    double dx, dy, dz, droll, dpitch, dyaw;
    GetTranslationAndEulerAngles(odom1_delta_fron_last_kf, dx, dy, dz, droll, dpitch, dyaw);
    double d_trans_m = sqrt(dx * dx + dy * dy + dz * dz);
    double d_rot_rad = sqrt(droll * droll + dpitch * dpitch + dyaw * dyaw);

    // Check if the keyframe is needed
    double th_tlans_m = cfg_d_keyframe_dist_threshold_m_;
    double th_rot_rad = cfg_d_keyframe_rot_threshold_deg_ * M_PI / 180.0;
    if (d_trans_m < th_tlans_m && d_rot_rad < th_rot_rad) {
        // Keyframe generation condition is not satisfied
        return;
    }

    // Check motion condition for keyframe: calibration operate when move fast.
    for(int odom1_idx = 0; odom1_idx < odom1_last_idx; odom1_idx++) {
        // Linear velocity slow check
        double vx = deq_odom1_accumed_.at(odom1_idx).motion(0);
        double vy = deq_odom1_accumed_.at(odom1_idx).motion(1);
        double vz = deq_odom1_accumed_.at(odom1_idx).motion(2);

        double liner_vel = sqrt(vx * vx + vy * vy + vz * vz);

        if (liner_vel < 0.1) { // TBD: threshold need to be adjusted
            DebugPrintWarn("[RosOdomBasedCalibration] Odom1 motion is not stable");
            // TBD: Pop until the motion is increased
            
            return;
        }

        // Angular velocity slow check
        double wx = deq_odom1_accumed_.at(odom1_idx).motion(3);
        double wy = deq_odom1_accumed_.at(odom1_idx).motion(4);
        double wz = deq_odom1_accumed_.at(odom1_idx).motion(5);

        double angular_vel = sqrt(wx * wx + wy * wy + wz * wz);

        if (angular_vel < 0.1) { // TBD: threshold need to be adjusted
            DebugPrintWarn("[RosOdomBasedCalibration] Odom1 motion is not stable");
            return;
        }
    }
}

void RosOdomBasedCalibration::FaultDataExclusion() {}

void RosOdomBasedCalibration::IterativeRotationEstimation() {}

// ROS Callbacks
void RosOdomBasedCalibration::CallbackOdom1Odom(const nav_msgs::Odometry::ConstPtr &msg) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = msg->header;
    pose_stamped.pose = msg->pose.pose;
    PoseStampedToInputQue(pose_stamped, deq_odom1_accumed_);
    return;
}

void RosOdomBasedCalibration::CallbackOdom1Twist(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    geometry_msgs::TwistStamped twist_stamped;
    twist_stamped.header = msg->header;
    twist_stamped.twist = msg->twist;
    TwistStampedToInputQue(twist_stamped, deq_odom1_accumed_);
    return;
}

void RosOdomBasedCalibration::CallbackOdom1Pose(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    PoseStampedToInputQue(*msg, deq_odom1_accumed_);
    return;
}
void RosOdomBasedCalibration::CallbackOdom2Odom(const nav_msgs::Odometry::ConstPtr &msg) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = msg->header;
    pose_stamped.pose = msg->pose.pose;
    PoseStampedToInputQue(pose_stamped, deq_odom2_accumed_);
    return;
}

void RosOdomBasedCalibration::CallbackOdom2Twist(const geometry_msgs::TwistStamped::ConstPtr &msg) {
    geometry_msgs::TwistStamped twist_stamped;
    twist_stamped.header = msg->header;
    twist_stamped.twist = msg->twist;
    TwistStampedToInputQue(twist_stamped, deq_odom2_accumed_);
    return;
}

void RosOdomBasedCalibration::CallbackOdom2Pose(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    PoseStampedToInputQue(*msg, deq_odom2_accumed_);
    return;
}

void RosOdomBasedCalibration::PoseStampedToInputQue(const geometry_msgs::PoseStamped &msg,
                                                    std::deque<OdomMotion> &deq) {
    static geometry_msgs::PoseStamped odom1_init = msg;
    static Eigen::Affine3d odom1_tf_init = GetAffine3dFromPose(odom1_init.pose);
    OdomMotion odom_motion_1_curr;
    odom_motion_1_curr.time_sec = msg.header.stamp.toSec();

    // Determine odometry from initial
    geometry_msgs::PoseStamped odom1_curr = msg;
    static geometry_msgs::PoseStamped odom1_prev = odom1_curr;
    Eigen::Affine3d odom1_tf_curr = GetAffine3dFromPose(odom1_curr.pose);

    Eigen::Affine3d odom1_tf_from_init = odom1_tf_init.inverse() * odom1_tf_curr;
    static Eigen::Affine3d odom1_tf_prev = odom1_tf_from_init;

    odom_motion_1_curr.odom = odom1_tf_from_init;

    // Determine motion from previous
    double delta_time_s = odom1_curr.header.stamp.toSec() - odom1_prev.header.stamp.toSec();
    Eigen::VectorXd motion = Eigen::VectorXd::Zero(6);
    if (delta_time_s > 0.0) {
        Eigen::Affine3d odom1_delta = odom1_tf_prev.inverse() * odom1_tf_from_init;
        double dx, dy, dz, droll, dpitch, dyaw;
        GetTranslationAndEulerAngles(odom1_delta, dx, dy, dz, droll, dpitch, dyaw);
        motion << dx / delta_time_s, dy / delta_time_s, dz / delta_time_s, droll / delta_time_s, dpitch / delta_time_s,
                dyaw / delta_time_s;
    }
    else {
        motion << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    }
    odom_motion_1_curr.motion = motion;

    // Push back
    deq.push_back(odom_motion_1_curr);
    QueLimiters(deq, cfg_i_que_input_max_size_);
    odom1_prev = odom1_curr;
    return;
}

void RosOdomBasedCalibration::TwistStampedToInputQue(const geometry_msgs::TwistStamped &msg,
                                                     std::deque<OdomMotion> &deq) {
    static geometry_msgs::TwistStamped odom2_prev = msg;
    geometry_msgs::TwistStamped odom2_curr = msg;

    // Determine delta odometry
    static Eigen::Affine3d odom2_accumed = Eigen::Affine3d::Identity();
    double delta_time_sec = odom2_curr.header.stamp.toSec() - odom2_prev.header.stamp.toSec();

    // Translation
    double vx = (odom2_curr.twist.linear.x + odom2_prev.twist.linear.x) / 2.0;
    double vy = (odom2_curr.twist.linear.y + odom2_prev.twist.linear.y) / 2.0;
    double vz = (odom2_curr.twist.linear.z + odom2_prev.twist.linear.z) / 2.0;

    double dx = vx * delta_time_sec;
    double dy = vy * delta_time_sec;
    double dz = vz * delta_time_sec;

    // Rotation
    double wx = (odom2_curr.twist.angular.x + odom2_prev.twist.angular.x) / 2.0;
    double wy = (odom2_curr.twist.angular.y + odom2_prev.twist.angular.y) / 2.0;
    double wz = (odom2_curr.twist.angular.z + odom2_prev.twist.angular.z) / 2.0;

    double droll = wx * delta_time_sec;
    double dpitch = wy * delta_time_sec;
    double dyaw = wz * delta_time_sec;

    // Get transformation
    Eigen::Affine3d odom2_delta;
    odom2_delta.setIdentity();
    GetTransformation(dx, dy, dz, droll, dpitch, dyaw, odom2_delta);
    odom2_accumed = odom2_accumed * odom2_delta;

    // Get motion
    Eigen::VectorXd motion = Eigen::VectorXd::Zero(6);
    motion << vx, vy, vz, wx, wy, wz;

    // OdomMotion
    OdomMotion odom2_motion;
    odom2_motion.time_sec = odom2_curr.header.stamp.toSec();
    odom2_motion.odom = odom2_accumed;
    odom2_motion.motion = motion;

    // Push back
    deq.push_back(odom2_motion);
    QueLimiters(deq, cfg_i_que_input_max_size_);
    odom2_prev = odom2_curr;
    return;
}

void RosOdomBasedCalibration::QueLimiters(std::deque<OdomMotion> &deq, int max_size) {
    // Limit the size of the queue
    while (deq.size() > max_size) {
        deq.pop_front();
    }
}

void RosOdomBasedCalibration::QueLimiters(std::deque<OdomPair> &deq, int max_size) {
    // Limit the size of the queue
    while (deq.size() > max_size) {
        deq.pop_front();
    }
}

void RosOdomBasedCalibration::GetTransformation(float x, float y, float z, float roll, float pitch, float yaw,
                                                Eigen::Affine3f &t) {
    Eigen::Affine3d t_d;
    GetTransformation((double)x, (double)y, (double)z, (double)roll, (double)pitch, (double)yaw, t_d);
    t = t_d.cast<float>();
}

void RosOdomBasedCalibration::GetTransformation(double x, double y, double z, double roll, double pitch, double yaw,
                                                Eigen::Affine3d &t) {
    double A = cosf64(yaw), B = sinf64(yaw), C = cosf64(pitch), D = sinf64(pitch), E = cosf64(roll), F = sinf64(roll),
           DE = D * E, DF = D * F;
    t(0, 0) = A * C;
    t(0, 1) = A * DF - B * E;
    t(0, 2) = B * F + A * DE;
    t(0, 3) = x;
    t(1, 0) = B * C;
    t(1, 1) = A * E + B * DF;
    t(1, 2) = B * DE - A * F;
    t(1, 3) = y;
    t(2, 0) = -D;
    t(2, 1) = C * F;
    t(2, 2) = C * E;
    t(2, 3) = z;
    t(3, 0) = 0;
    t(3, 1) = 0;
    t(3, 2) = 0;
    t(3, 3) = 1;
}

void RosOdomBasedCalibration::PoseToAffine3d(const geometry_msgs::Pose &pose, Eigen::Affine3d &t) {
    Eigen::Vector3d translation = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    t.setIdentity();
    t.translate(translation);
    t.rotate(q);
}

Eigen::Affine3d RosOdomBasedCalibration::GetAffine3dFromPose(const geometry_msgs::Pose &pose) {
    Eigen::Affine3d t;
    t.setIdentity();
    PoseToAffine3d(pose, t);
    return t;
}

void RosOdomBasedCalibration::GetTranslationAndEulerAngles(const Eigen::Affine3d &t, double &x, double &y, double &z,
                                                           double &roll, double &pitch, double &yaw) {
    x = t(0, 3);
    y = t(1, 3);
    z = t(2, 3);
    roll = atan2f64(t(2, 1), t(2, 2));
    pitch = asinf64(-t(2, 0));
    yaw = atan2f64(t(1, 0), t(0, 0));
}

Eigen::Affine3d RosOdomBasedCalibration::GetIntpAffine3d(const Eigen::Affine3d &t1, double ratio) {
    // Get translation and euler angles
    double x1, y1, z1, roll1, pitch1, yaw1;
    GetTranslationAndEulerAngles(t1, x1, y1, z1, roll1, pitch1, yaw1);

    // Get interpolated transformation
    double x2 = x1 * ratio;
    double y2 = y1 * ratio;
    double z2 = z1 * ratio;
    double roll2 = roll1 * ratio;
    double pitch2 = pitch1 * ratio;
    double yaw2 = yaw1 * ratio;

    Eigen::Affine3d t;
    GetTransformation(x2, y2, z2, roll2, pitch2, yaw2, t);
    return t;
}

Eigen::Affine3d RosOdomBasedCalibration::GetIntpAffine3d(const Eigen::Affine3d &t1, const Eigen::Affine3d &t2,
                                                         double ratio) {
    Eigen::Affine3d t_1to2 = t1.inverse() * t2;
    Eigen::Affine3d t_1tor = GetIntpAffine3d(t_1to2, ratio);
    Eigen::Affine3d t_r = t1 * t_1tor;
    return t_r;
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