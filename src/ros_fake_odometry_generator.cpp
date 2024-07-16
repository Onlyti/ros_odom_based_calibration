/**
 * @file ros_fake_odometry_generator.cpp
 * @author Jiwon Seok (pauljiwon96@gmail.com)
 * @brief This ros node purpose is to simulate fake odometry data for calibration purpose. Recieve the odometry data
 * from the topic and publish the fake odometry data.
 * @version 0.1
 * @date 2024-07-16
 *
 * @copyright Copyright (c) 2024
 *
 */

// Headers
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include "ini_handler_cpp/c_ini.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

// Global variables
ros::Publisher pub_odom;
nav_msgs::Odometry input_odom;
bool is_odom_received = false;

// Callback function
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    input_odom = *msg;
    is_odom_received = true;
}

// Main function
int main(int argc, char** argv) {
    // Initialize ROS node
    ros::init(argc, argv, "ros_fake_odometry_generator");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // Recieve ini path fron ros parameter
    std::string ini_path;
    pnh.getParam("ini_path", ini_path);
    ROS_INFO_STREAM("ini_path: " << ini_path);
    CINI_H ini_h;
    ini_h.Init(ini_path);

    // Parse ini the calibration parameters
    double x_offset_m, y_offset_m, z_offset_m;
    double roll_offset_deg, pitch_offset_deg, yaw_offset_deg;
    double roll_offset_rad, pitch_offset_rad, yaw_offset_rad;

    ini_h.ParseConfig("calibration", "x_offset_m", x_offset_m);
    ini_h.ParseConfig("calibration", "y_offset_m", y_offset_m);
    ini_h.ParseConfig("calibration", "z_offset_m", z_offset_m);
    ini_h.ParseConfig("calibration", "roll_offset_deg", roll_offset_deg);
    ini_h.ParseConfig("calibration", "pitch_offset_deg", pitch_offset_deg);
    ini_h.ParseConfig("calibration", "yaw_offset_deg", yaw_offset_deg);

    ROS_INFO_STREAM("x_offset_m: " << x_offset_m);
    ROS_INFO_STREAM("y_offset_m: " << y_offset_m);
    ROS_INFO_STREAM("z_offset_m: " << z_offset_m);
    ROS_INFO_STREAM("roll_offset_deg: " << roll_offset_deg);
    ROS_INFO_STREAM("pitch_offset_deg: " << pitch_offset_deg);
    ROS_INFO_STREAM("yaw_offset_deg: " << yaw_offset_deg);

    roll_offset_rad = roll_offset_deg * M_PI / 180.0;
    pitch_offset_rad = pitch_offset_deg * M_PI / 180.0;
    yaw_offset_rad = yaw_offset_deg * M_PI / 180.0;

    // Generate Eigen::Affine3d
    Eigen::Affine3d offset = Eigen::Affine3d::Identity();
    offset.translation() << x_offset_m, y_offset_m, z_offset_m;
    offset.rotate(Eigen::AngleAxisd(yaw_offset_rad, Eigen::Vector3d::UnitZ()));
    offset.rotate(Eigen::AngleAxisd(pitch_offset_rad, Eigen::Vector3d::UnitY()));
    offset.rotate(Eigen::AngleAxisd(roll_offset_rad, Eigen::Vector3d::UnitX()));

    // Publisher
    pub_odom = nh.advertise<nav_msgs::Odometry>("/fake_odom", 1);

    // Subscriber
    std::string odom_topic;
    ini_h.ParseConfig("ros", "odom_topic", odom_topic);
    ROS_INFO_STREAM("odom_topic: " << odom_topic);
    ros::Subscriber sub_odom = nh.subscribe(odom_topic.c_str(), 1, odomCallback);

    // Loop
    ros::Rate loop_rate(1000);
    while (ros::ok()) {
        if (ini_h.IsFileUpdated() == true) {
            // Update calibration parameters
            ini_h.ParseConfig("calibration", "x_offset_m", x_offset_m);
            ini_h.ParseConfig("calibration", "y_offset_m", y_offset_m);
            ini_h.ParseConfig("calibration", "z_offset_m", z_offset_m);
            ini_h.ParseConfig("calibration", "roll_offset_deg", roll_offset_deg);
            ini_h.ParseConfig("calibration", "pitch_offset_deg", pitch_offset_deg);
            ini_h.ParseConfig("calibration", "yaw_offset_deg", yaw_offset_deg);

            ROS_INFO_STREAM("x_offset_m: " << x_offset_m);
            ROS_INFO_STREAM("y_offset_m: " << y_offset_m);
            ROS_INFO_STREAM("z_offset_m: " << z_offset_m);
            ROS_INFO_STREAM("roll_offset_deg: " << roll_offset_deg);
            ROS_INFO_STREAM("pitch_offset_deg: " << pitch_offset_deg);
            ROS_INFO_STREAM("yaw_offset_deg: " << yaw_offset_deg);

            roll_offset_rad = roll_offset_deg * M_PI / 180.0;
            pitch_offset_rad = pitch_offset_deg * M_PI / 180.0;
            yaw_offset_rad = yaw_offset_deg * M_PI / 180.0;

            // Update Eigen::Affine3d
            offset = Eigen::Affine3d::Identity();
            offset.translation() << x_offset_m, y_offset_m, z_offset_m;
            offset.rotate(Eigen::AngleAxisd(yaw_offset_rad, Eigen::Vector3d::UnitZ()));
            offset.rotate(Eigen::AngleAxisd(pitch_offset_rad, Eigen::Vector3d::UnitY()));
            offset.rotate(Eigen::AngleAxisd(roll_offset_rad, Eigen::Vector3d::UnitX()));

            // Update subscriber
            ini_h.ParseConfig("ros", "odom_topic", odom_topic);
            ROS_INFO_STREAM("odom_topic: " << odom_topic);
            sub_odom = nh.subscribe(odom_topic.c_str(), 1, odomCallback);
        }
        if (is_odom_received) {
            // Generate fake odometry
            nav_msgs::Odometry fake_odom = input_odom;
            Eigen::Affine3d odom = Eigen::Affine3d::Identity();
            odom.translation() << input_odom.pose.pose.position.x, input_odom.pose.pose.position.y,
                    input_odom.pose.pose.position.z;
            odom.rotate(Eigen::Quaterniond(input_odom.pose.pose.orientation.w, input_odom.pose.pose.orientation.x,
                                           input_odom.pose.pose.orientation.y, input_odom.pose.pose.orientation.z));
            odom = odom * offset;

            fake_odom.pose.pose.position.x = odom.translation().x();
            fake_odom.pose.pose.position.y = odom.translation().y();
            fake_odom.pose.pose.position.z = odom.translation().z();
            Eigen::Quaterniond q(odom.rotation());
            fake_odom.pose.pose.orientation.w = q.w();
            fake_odom.pose.pose.orientation.x = q.x();
            fake_odom.pose.pose.orientation.y = q.y();
            fake_odom.pose.pose.orientation.z = q.z();

            // Publish fake odometry
            pub_odom.publish(fake_odom);
            is_odom_received = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
