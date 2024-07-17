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
      cfg_d_keyframe_slow_threshold_mps_(0.0),
      cfg_d_time_delay_odom1_to_odom2_sec_(0.0),
      cfg_d_calib_update_rate_(0.0),
      cfg_d_cal_t_x_m_(0.0),
      cfg_d_cal_t_y_m_(0.0),
      cfg_d_cal_t_z_m_(0.0),
      cfg_b_cal_use_predef_roll_(false),
      cfg_b_cal_use_predef_pitch_(false),
      cfg_b_cal_use_predef_yaw_(false),
      cfg_d_cal_r_roll_deg_(0.0),
      cfg_d_cal_r_pitch_deg_(0.0),
      cfg_d_cal_r_yaw_deg_(0.0),
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

    return true;
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
        if (ini_handler_.ParseConfig("Config", "cfg_d_keyframe_slow_threshold_mps_",
                                     cfg_d_keyframe_slow_threshold_mps_) == false) {
            DebugPrintError("[RosOdomBasedCalibration] Failed to parse cfg_d_keyframe_slow_threshold_mps_");
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
        if (ini_handler_.ParseConfig("Config", "cfg_d_calib_update_rate_", cfg_d_calib_update_rate_) == false) {
            DebugPrintError("[RosOdomBasedCalibration] Failed to parse cfg_d_calib_update_rate_");
            return false;
        }
        if (cfg_d_calib_update_rate_ <= 0.0) {
            DebugPrintError("[RosOdomBasedCalibration] cfg_d_calib_update_rate_ should be positive value");
            return false;
        }

        // Predefined translation calibration parameters
        if (ini_handler_.ParseConfig("Calib", "cfg_d_cal_t_x_m_", cfg_d_cal_t_x_m_) == false) {
            DebugPrintError("[RosOdomBasedCalibration] Failed to parse cfg_d_cal_t_x_m_");
            return false;
        }
        if (ini_handler_.ParseConfig("Calib", "cfg_d_cal_t_y_m_", cfg_d_cal_t_y_m_) == false) {
            DebugPrintError("[RosOdomBasedCalibration] Failed to parse cfg_d_cal_t_y_m_");
            return false;
        }
        if (ini_handler_.ParseConfig("Calib", "cfg_d_cal_t_z_m_", cfg_d_cal_t_z_m_) == false) {
            DebugPrintError("[RosOdomBasedCalibration] Failed to parse cfg_d_cal_t_z_m_");
            return false;
        }
        if (ini_handler_.ParseConfig("Calib", "cfg_b_cal_use_predef_roll_", cfg_b_cal_use_predef_roll_) == false) {
            DebugPrintError("[RosOdomBasedCalibration] Failed to parse cfg_b_cal_use_predef_roll_");
            return false;
        }
        if (ini_handler_.ParseConfig("Calib", "cfg_b_cal_use_predef_pitch_", cfg_b_cal_use_predef_pitch_) == false) {
            DebugPrintError("[RosOdomBasedCalibration] Failed to parse cfg_b_cal_use_predef_pitch_");
            return false;
        }
        if (ini_handler_.ParseConfig("Calib", "cfg_b_cal_use_predef_yaw_", cfg_b_cal_use_predef_yaw_) == false) {
            DebugPrintError("[RosOdomBasedCalibration] Failed to parse cfg_b_cal_use_predef_yaw_");
            return false;
        }
        if (ini_handler_.ParseConfig("Calib", "cfg_d_cal_r_roll_deg_", cfg_d_cal_r_roll_deg_) == false) {
            DebugPrintError("[RosOdomBasedCalibration] Failed to parse cfg_d_cal_r_roll_deg_");
            return false;
        }
        if (ini_handler_.ParseConfig("Calib", "cfg_d_cal_r_pitch_deg_", cfg_d_cal_r_pitch_deg_) == false) {
            DebugPrintError("[RosOdomBasedCalibration] Failed to parse cfg_d_cal_r_pitch_deg_");
            return false;
        }
        if (ini_handler_.ParseConfig("Calib", "cfg_d_cal_r_yaw_deg_", cfg_d_cal_r_yaw_deg_) == false) {
            DebugPrintError("[RosOdomBasedCalibration] Failed to parse cfg_d_cal_r_yaw_deg_");
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
    RotationEstimation();
    PublishCalibrationResult();
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

            if (odom1_front_time_sec >= odom2_front_time_sec && odom1_front_time_sec < odom2_back_time_sec) {
                double odom2_second_time_sec = deq_odom2_accumed_.at(1).time_sec;

                // Check if the time of odom1 is in between the time of odom2. Verify time order like.
                // [ Odom2.front <= Odom1.front < Odom1.seoncd ]
                while (odom1_front_time_sec >= odom2_second_time_sec) {
                    deq_odom2_accumed_.pop_front(); // Reject unsynced odom2 data

                    if (deq_odom2_accumed_.size() < 2) {
                        DebugPrintWarn("[RosOdomBasedCalibration] Odom2 time is not synchronized");
                        return;
                    }

                    odom2_front_time_sec = deq_odom2_accumed_.front().time_sec;
                    odom2_second_time_sec = deq_odom2_accumed_.at(1).time_sec;
                }

                b_start_condition = true;
                time_last_kf_sec_ = odom1_front_time_sec;
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
                if (odom1_front_time_sec < odom2_front_time_sec) deq_odom1_accumed_.pop_front();
                return;
            }
        }
    }

    /* Keyframe Selection */
    // Keyframe generation condition
    // Find current keyframe time satisify that the odom1 last time smaller then odom2 last time (for interpolation)
    // [ odom2.second_last < curr_kf < odom2.last ]
    double odom1_front_time = deq_odom1_accumed_.front().time_sec;
    double odom1_last_time_sec = odom1_front_time;
    int curr_kf_odom1_idx = 0;
    int curr_kf_odom2_last_idx = 0;
    double odom2_last_time_sec = deq_odom2_accumed_.back().time_sec;
    for (int odom1_idx = deq_odom1_accumed_.size() - 1; odom1_idx >= 0; odom1_idx--) {
        double odom1_curr_time_sec = deq_odom1_accumed_.at(odom1_idx).time_sec;
        if (odom1_curr_time_sec < odom2_last_time_sec) { // Curr kf is in between the odom2 last and second last time.
            for (int odom2_idx = deq_odom2_accumed_.size() - 1; odom2_idx >= 1; odom2_idx--) {
                double odom2_second_last_time_sec = deq_odom2_accumed_.at(odom2_idx - 1).time_sec;
                double odom2_last_time_sec = deq_odom2_accumed_.at(odom2_idx).time_sec;
                if (odom1_curr_time_sec >= odom2_second_last_time_sec && odom1_curr_time_sec < odom2_last_time_sec) {
                    curr_kf_odom2_last_idx = odom2_idx;
                    break;
                }
            }
            odom1_last_time_sec = deq_odom1_accumed_.at(odom1_idx).time_sec;
            curr_kf_odom1_idx = odom1_idx;
            break;
        }
    }
    if (curr_kf_odom1_idx == 0 || curr_kf_odom2_last_idx == 0) {
        DebugPrintWarn("[RosOdomBasedCalibration] Odom1 and Odom2 time is not synchronized");
        return;
    }
    // Check if the odom1 last time is found
    if (odom1_last_time_sec <= odom1_front_time) {
        DebugPrintWarn("[RosOdomBasedCalibration] Odom1 last time is not found");
        return;
    }

    // Check difference between the last keyframe and the last odom1
    Eigen::Affine3d odom1_at_curr_kf_time = deq_odom1_accumed_.at(curr_kf_odom1_idx).odom;
    Eigen::Affine3d odom1_last_to_curr_kf = last_keyframe_odom1_.inverse() * odom1_at_curr_kf_time;
    double dx, dy, dz, droll, dpitch, dyaw;
    GetTranslationAndEulerAngles(odom1_last_to_curr_kf, dx, dy, dz, droll, dpitch, dyaw);
    double d_trans_m = sqrt(dx * dx + dy * dy + dz * dz);
    double d_rot_rad = sqrt(droll * droll + dpitch * dpitch + dyaw * dyaw);

    // Check if the keyframe is needed
    double th_tlans_m = cfg_d_keyframe_dist_threshold_m_;
    double th_rot_rad = cfg_d_keyframe_rot_threshold_deg_ * M_PI / 180.0;
    if (d_trans_m < th_tlans_m && d_rot_rad < th_rot_rad) {
        // Keyframe generation condition is not satisfied. Wait for the odom1 is moved enough.
        // DebugPrintWarn("[RosOdomBasedCalibration] Odom1 motion is not enough");
        return;
    }

    // Keyframe-based delta odom generation
    Eigen::Affine3d odom2_at_curr_kf_time = Eigen::Affine3d::Identity();
    // Determine odom2 at current kf time
    double time_curr_kf_sec = deq_odom1_accumed_.at(curr_kf_odom1_idx).time_sec;
    double time_curr_odom2_last = deq_odom2_accumed_.at(curr_kf_odom2_last_idx).time_sec;
    double time_curr_odom2_last_minus_one = deq_odom2_accumed_.at(curr_kf_odom2_last_idx - 1).time_sec;
    double ratio = (time_curr_kf_sec - time_curr_odom2_last_minus_one) /
                   (time_curr_odom2_last - time_curr_odom2_last_minus_one);

    odom2_at_curr_kf_time = GetIntpAffine3d(deq_odom2_accumed_.at(curr_kf_odom2_last_idx - 1).odom,
                                            deq_odom2_accumed_.at(curr_kf_odom2_last_idx).odom, ratio);

    Eigen::Affine3d odom2_last_to_curr_kf = last_keyframe_odom2_.inverse() * odom2_at_curr_kf_time;

    // Keyframe pair generation
    OdomPair odom_pair;
    odom_pair.time_sec = time_last_kf_sec_;
    odom_pair.delta_time_sec = time_curr_kf_sec - time_last_kf_sec_;
    odom_pair.odom1 = odom1_last_to_curr_kf;
    odom_pair.odom2 = odom2_last_to_curr_kf;
    odom_pair.t_error_m = FLT_MAX;
    odom_pair.r_error_rad = FLT_MAX;

    // Update keyframe
    last_keyframe_odom1_ = odom1_at_curr_kf_time;
    last_keyframe_odom2_ = odom2_at_curr_kf_time;
    time_last_kf_sec_ = deq_odom1_accumed_.at(curr_kf_odom1_idx).time_sec;

    // Check motion condition for keyframe: calibration operate when move fast.
    bool is_motion_validate = true;
    for (int odom1_idx = 0; odom1_idx < curr_kf_odom1_idx; odom1_idx++) {
        // Linear velocity slow check
        double vx = deq_odom1_accumed_.at(odom1_idx).motion(0);
        double vy = deq_odom1_accumed_.at(odom1_idx).motion(1);
        double vz = deq_odom1_accumed_.at(odom1_idx).motion(2);

        double liner_vel = sqrt(vx * vx + vy * vy + vz * vz);

        if (liner_vel < cfg_d_keyframe_slow_threshold_mps_) {
            DebugPrintWarn("[RosOdomBasedCalibration] Odom1 motion is not stable");
            is_motion_validate = false;
        }
    }
    // Clear input que
    deq_odom1_accumed_.erase(deq_odom1_accumed_.begin(), deq_odom1_accumed_.begin() + curr_kf_odom1_idx - 1);
    deq_odom2_accumed_.erase(deq_odom2_accumed_.begin(), deq_odom2_accumed_.begin() + curr_kf_odom2_last_idx - 1);

    if (is_motion_validate == false) return;

    // Keyframe pair push back
    deq_odom_pair_.push_back(odom_pair);
}

void RosOdomBasedCalibration::FaultDataExclusion() {
    // Iterate through the deque from back to front
    for (auto it = deq_odom_pair_.begin(); it != deq_odom_pair_.end(); it++) {
        // Calculate rotation differences
        double roll_odom1, pitch_odom1, yaw_odom1;
        double roll_odom2, pitch_odom2, yaw_odom2;
        double dx, dy, dz;

        // Extract Euler angles from odometry data
        GetTranslationAndEulerAngles(it->odom1, dx, dy, dz, roll_odom1, pitch_odom1, yaw_odom1);
        GetTranslationAndEulerAngles(it->odom2, dx, dy, dz, roll_odom2, pitch_odom2, yaw_odom2);
        double roll_diff = fabs(roll_odom1 - roll_odom2);
        double pitch_diff = fabs(pitch_odom1 - pitch_odom2);
        double yaw_diff = fabs(yaw_odom1 - yaw_odom2);

        // Normalize the angle difference
        if (roll_diff > M_PI) roll_diff = fabs(roll_diff - 2 * M_PI);
        if (pitch_diff > M_PI) pitch_diff = fabs(pitch_diff - 2 * M_PI);
        if (yaw_diff > M_PI) yaw_diff = fabs(yaw_diff - 2 * M_PI);

        // Store the maximum rotation difference as the rotation error
        it->r_error_rad = sqrt(roll_diff * roll_diff + pitch_diff * pitch_diff + yaw_diff * yaw_diff);

        // Perform the rotation fault check
        double fde_rot_err_th_rad = cfg_d_fde_max_rotation_error_threshold_deg_ * M_PI / 180.0;
        if (it->r_error_rad > fde_rot_err_th_rad) {
            DebugPrintWarn("[RosOdomBasedCalibration] Detected rotation fault");
            continue; // Move to the next element
        }

        // Calculate translation difference
        Eigen::Vector3d translation_odom1 = it->odom1.translation();
        Eigen::Vector3d translation_odom2 = it->odom2.translation();
        double translation_diff = translation_odom1.norm() - translation_odom2.norm();
        // Determine the rotation angle of rotation axis
        double rot_angle_rad = sqrt(roll_odom1 * roll_odom1 + pitch_odom1 * pitch_odom1 + yaw_odom1 * yaw_odom1);
        double cal_t_length = sqrt(cfg_d_cal_t_x_m_ * cfg_d_cal_t_x_m_ + cfg_d_cal_t_y_m_ * cfg_d_cal_t_y_m_ +
                                   cfg_d_cal_t_z_m_ * cfg_d_cal_t_z_m_);
        double t_error = translation_diff - 2.0 * fabs(sin(rot_angle_rad / 2.0)) * cal_t_length;

        it->t_error_m = t_error;

        // Perform the translation fault check
        if (it->t_error_m > cfg_d_fde_max_translation_error_threshold_m_) {
            DebugPrintWarn("[RosOdomBasedCalibration] Detected translation fault");
            continue; // Move to the next element
        }

        // If both checks pass, add the odom pair to valid list
        deq_valid_odom_pair_.push_back(*it);
    }

    // Clear the original deque
    deq_odom_pair_.clear();

    // Limit the size of the valid odom pair deque
    QueLimiters(deq_valid_odom_pair_, cfg_i_que_keyframe_max_size_);

    return;
}

void RosOdomBasedCalibration::RotationEstimation() {
    // Ensure we have valid odometry pairs to process
    if (deq_valid_odom_pair_.empty()) {
        DebugPrintWarn("[RosOdomBasedCalibration] No valid odometry pairs available for rotation estimation.");
        return;
    }

    // Reduce degree of freedom
    Eigen::Affine3d last_esti_r_p_rot = Eigen::Affine3d::Identity();
    GetTransformation(0.0, 0.0, 0.0, d_estimated_roll_rad_, d_estimated_pitch_rad_, 0.0, last_esti_r_p_rot);
    Eigen::Affine3d last_esti_r_y_rot = Eigen::Affine3d::Identity();
    GetTransformation(0.0, 0.0, 0.0, d_estimated_roll_rad_, 0.0, d_estimated_yaw_rad_, last_esti_r_y_rot);
    Eigen::Affine3d last_esti_p_y_rot = Eigen::Affine3d::Identity();
    GetTransformation(0.0, 0.0, 0.0, 0.0, d_estimated_pitch_rad_, d_estimated_yaw_rad_, last_esti_p_y_rot);

    // Generate reduced dof odom pairs
    for (auto &odom_pair : deq_valid_odom_pair_) {
        odom_pair.odom2_for_roll = odom_pair.odom2 * last_esti_p_y_rot.inverse();
        odom_pair.odom2_for_pitch = odom_pair.odom2 * last_esti_r_y_rot.inverse();
        odom_pair.odom2_for_yaw = odom_pair.odom2 * last_esti_r_p_rot.inverse();
    }

    // Least Sqaure
    Eigen::MatrixXd A_for_roll = Eigen::MatrixXd::Zero(2 * deq_valid_odom_pair_.size(), 2);
    Eigen::VectorXd B_for_roll = Eigen::VectorXd::Zero(2 * deq_valid_odom_pair_.size());
    Eigen::MatrixXd A_for_pitch = Eigen::MatrixXd::Zero(2 * deq_valid_odom_pair_.size(), 2);
    Eigen::VectorXd B_for_pitch = Eigen::VectorXd::Zero(2 * deq_valid_odom_pair_.size());
    Eigen::MatrixXd A_for_yaw = Eigen::MatrixXd::Zero(2 * deq_valid_odom_pair_.size(), 2);
    Eigen::VectorXd B_for_yaw = Eigen::VectorXd::Zero(2 * deq_valid_odom_pair_.size());

    // Fill the matrix A and vector B
    int idx = 0;
    double roll_esti, pitch_esti, yaw_esti;
    for (const auto &odom_pair : deq_valid_odom_pair_) {
        auto &od1 = odom_pair.odom1;
        double od1_x, od1_y, od1_z, od1_roll, od1_pitch, od1_yaw;
        GetTranslationAndEulerAngles(od1, od1_x, od1_y, od1_z, od1_roll, od1_pitch, od1_yaw);
        auto &od2_r = odom_pair.odom2_for_roll;
        auto &od2_p = odom_pair.odom2_for_pitch;
        auto &od2_y = odom_pair.odom2_for_yaw;
        // Roll
        {
            double dx, dy, dz, droll, dpitch, dyaw;
            GetTranslationAndEulerAngles(od2_r, dx, dy, dz, droll, dpitch, dyaw);
            A_for_roll(idx, 0) = dy;  // y
            A_for_roll(idx, 1) = -dz; // z
            B_for_roll(idx) = (cos(od1_roll) - 1.0) * cfg_d_cal_t_y_m_ - sin(od1_roll) * cfg_d_cal_t_z_m_ + od1_y;
            A_for_roll(idx + 1, 0) = dz; // z
            A_for_roll(idx + 1, 1) = dy; // y
            B_for_roll(idx + 1) = sin(od1_roll) * cfg_d_cal_t_y_m_ + (cos(od1_roll) - 1.0) * cfg_d_cal_t_z_m_ + od1_z;
        }

        // Pitch
        {
            double dx, dy, dz, droll, dpitch, dyaw;
            GetTranslationAndEulerAngles(od2_p, dx, dy, dz, droll, dpitch, dyaw);
            A_for_pitch(idx, 0) = dx;  // x
            A_for_pitch(idx, 1) = -dz; // z
            B_for_pitch(idx) = (cos(-od1_pitch) - 1.0) * cfg_d_cal_t_x_m_ - sin(-od1_pitch) * cfg_d_cal_t_z_m_ + od1_x;
            A_for_pitch(idx + 1, 0) = dz; // z
            A_for_pitch(idx + 1, 1) = dx; // x
            B_for_pitch(idx + 1) =
                    sin(-od1_pitch) * cfg_d_cal_t_x_m_ + (cos(-od1_pitch) - 1.0) * cfg_d_cal_t_z_m_ + od1_z;
        }

        // Yaw
        {
            double dx, dy, dz, droll, dpitch, dyaw;
            GetTranslationAndEulerAngles(od2_y, dx, dy, dz, droll, dpitch, dyaw);
            A_for_yaw(idx, 0) = dx;  // x
            A_for_yaw(idx, 1) = -dy; // y
            B_for_yaw(idx) = (cos(od1_yaw) - 1.0) * cfg_d_cal_t_x_m_ - sin(od1_yaw) * cfg_d_cal_t_y_m_ + od1_x;
            A_for_yaw(idx + 1, 0) = dy; // y
            A_for_yaw(idx + 1, 1) = dx; // x
            B_for_yaw(idx + 1) = sin(od1_yaw) * cfg_d_cal_t_x_m_ + (cos(od1_yaw) - 1.0) * cfg_d_cal_t_y_m_ + od1_y;
        }
        idx = idx + 2;
    }

    // Solve for the estimated roll, pitch, and yaw using least squares
    double up_rate = cfg_d_calib_update_rate_;
    Eigen::Vector2d cos_sin_roll = A_for_roll.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B_for_roll);
    double esti_roll_rad = atan2(cos_sin_roll(1), cos_sin_roll(0)); // atan2(sin, cos)
    if (cfg_b_cal_use_predef_roll_ == true)
        d_estimated_roll_rad_ = cfg_d_cal_r_roll_deg_ * M_PI / 180.0;
    else
        d_estimated_roll_rad_ = up_rate * esti_roll_rad + (1 - up_rate) * d_estimated_roll_rad_;

    Eigen::Vector2d cos_sin_pitch = A_for_pitch.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B_for_pitch);
    double esti_pitch_rad = -atan2(cos_sin_pitch(1), cos_sin_pitch(0)); // atan2(sin, cos)
    if (cfg_b_cal_use_predef_pitch_ == true)
        d_estimated_pitch_rad_ = cfg_d_cal_r_pitch_deg_ * M_PI / 180.0;
    else
        d_estimated_pitch_rad_ = up_rate * esti_pitch_rad + (1 - up_rate) * d_estimated_pitch_rad_;

    Eigen::Vector2d cos_sin_yaw = A_for_yaw.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B_for_yaw);
    double esti_yaw_rad = atan2(cos_sin_yaw(1), cos_sin_yaw(0)); // atan2(sin, cos)
    if (cfg_b_cal_use_predef_yaw_ == true)
        d_estimated_yaw_rad_ = cfg_d_cal_r_yaw_deg_ * M_PI / 180.0;
    else
        d_estimated_yaw_rad_ = up_rate * esti_yaw_rad + (1 - up_rate) * d_estimated_yaw_rad_;

    // Print the estimated roll, pitch, and yaw
    double estimated_roll_deg = d_estimated_roll_rad_ * 180.0 / M_PI;
    double estimated_pitch_deg = d_estimated_pitch_rad_ * 180.0 / M_PI;
    double estimated_yaw_deg = d_estimated_yaw_rad_ * 180.0 / M_PI;

    DebugPrintInfo("[RosOdomBasedCalibration] Estimated roll: " + std::to_string(estimated_roll_deg) + " deg");
    DebugPrintInfo("[RosOdomBasedCalibration] Estimated pitch: " + std::to_string(estimated_pitch_deg) + " deg");
    DebugPrintInfo("[RosOdomBasedCalibration] Estimated yaw: " + std::to_string(estimated_yaw_deg) + " deg\n");
}

void RosOdomBasedCalibration::PublishCalibrationResult() {
    // Publish the calibration result
    Eigen::Affine3d last_esti_calib_tf = Eigen::Affine3d::Identity();
    GetTransformation(cfg_d_cal_t_x_m_, cfg_d_cal_t_y_m_, cfg_d_cal_t_z_m_, d_estimated_roll_rad_,
                      d_estimated_pitch_rad_, d_estimated_yaw_rad_, last_esti_calib_tf);
    geometry_msgs::PoseStamped calib_odom1_to_odom2;
    calib_odom1_to_odom2.header.stamp = ros::Time::now();
    calib_odom1_to_odom2.pose = GetPoseFromAffine3d(last_esti_calib_tf);
    rospub_calib_odom1_to_odom2_.publish(calib_odom1_to_odom2);

    // Publish the translation and rotation errors
    std_msgs::Float64 fde_translation_error_m;
    fde_translation_error_m.data = FLT_MAX;
    std_msgs::Float64 fde_rotation_error_deg;
    fde_rotation_error_deg.data = FLT_MAX;
    if (deq_valid_odom_pair_.empty() == false) {
        fde_translation_error_m.data = deq_valid_odom_pair_.back().t_error_m;
        fde_rotation_error_deg.data = deq_valid_odom_pair_.back().r_error_rad * 180.0 / M_PI;
    }
    rospub_fde_translation_error_m_.publish(fde_translation_error_m);
    rospub_fde_rotation_error_deg_.publish(fde_rotation_error_deg);
}
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

geometry_msgs::Pose RosOdomBasedCalibration::GetPoseFromAffine3d(const Eigen::Affine3d &t) {
    geometry_msgs::Pose pose;
    pose.position.x = t(0, 3);
    pose.position.y = t(1, 3);
    pose.position.z = t(2, 3);
    Eigen::Quaterniond q(t.rotation());
    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();

    return pose;
}

// Main
int main(int argc, char **argv) {
    ros::init(argc, argv, "ros_odom_based_calibration");
    RosOdomBasedCalibration ros_odom_based_calibration;

    double frequency = 1000.0;
    ros::Rate loop_rate(frequency);
    while (ros::ok()) {
        ros_odom_based_calibration.Run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}