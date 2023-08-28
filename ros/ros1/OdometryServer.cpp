// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <Eigen/Core>
#include <vector>

// KISS-ICP-ROS
#include "OdometryServer.hpp"
#include "Utils.hpp"

// KISS-ICP
#include "kiss_icp/pipeline/KissICP.hpp"

// ROS
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

#include <fstream>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/transform_listener.h>


namespace kiss_icp_ros {

OdometryServer::OdometryServer(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh), tf_buffer_(ros::Duration(2)), tf_listener_(tf_buffer_) {
    pnh_.param("child_frame", child_frame_, child_frame_);
    pnh_.param("odom_frame", odom_frame_, odom_frame_);
    pnh_.param("max_range", config_.max_range, config_.max_range);
    pnh_.param("min_range", config_.min_range, config_.min_range);
    pnh_.param("deskew", config_.deskew, config_.deskew);
    pnh_.param("voxel_size", config_.voxel_size, config_.max_range / 100.0);
    pnh_.param("max_points_per_voxel", config_.max_points_per_voxel, config_.max_points_per_voxel);
    pnh_.param("initial_threshold", config_.initial_threshold, config_.initial_threshold);
    pnh_.param("min_motion_th", config_.min_motion_th, config_.min_motion_th);
    if (config_.max_range < config_.min_range) {
        ROS_WARN("[WARNING] max_range is smaller than min_range, setting min_range to 0.0");
        config_.min_range = 0.0;
    }

    // Construct the main KISS-ICP odometry node
    odometry_ = kiss_icp::pipeline::KissICP(config_);

    // Initialize subscribers
    pointcloud_sub_ = nh_.subscribe<const sensor_msgs::PointCloud2 &>(
        "pointcloud_topic", queue_size_, &OdometryServer::RegisterFrame, this);

    odom_sub_ = nh_.subscribe<const nav_msgs::Odometry &>(
        "/odom_for_kiss", queue_size_, &OdometryServer::RegisterOdometry, this);    

    std::cout << "pointcloud_sub_ = " << pointcloud_sub_.getTopic() << std::endl;

    std::cout << "child frame " << child_frame_ << std::endl;
    // odom_service = nh_.advertiseService("odom_kiss", &odomSericeCallback, this); //m
    

    // Initialize publishers
    odom_publisher_ = pnh_.advertise<nav_msgs::Odometry>("odometry", queue_size_);
    frame_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("frame", queue_size_);
    kpoints_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("keypoints", queue_size_);
    local_map_publisher_ = pnh_.advertise<sensor_msgs::PointCloud2>("local_map", queue_size_);

    odom_kiss_publisher_ = pnh_.advertise<nav_msgs::Odometry>("/odom_from_kiss", queue_size_);

    // Initialize trajectory publisher
    path_msg_.header.frame_id = odom_frame_;
    traj_publisher_ = pnh_.advertise<nav_msgs::Path>("trajectory", queue_size_);

    // Broadcast a static transformation that links with identity the specified base link to the
    // pointcloud_frame, basically to always be able to visualize the frame in rviz
    if (child_frame_ != "base_link") {
        static tf2_ros::StaticTransformBroadcaster br;
        geometry_msgs::TransformStamped alias_transform_msg;
        alias_transform_msg.header.stamp = ros::Time::now();
        alias_transform_msg.transform.translation.x = 0.0;
        alias_transform_msg.transform.translation.y = 0.0;
        alias_transform_msg.transform.translation.z = 0.0;
        alias_transform_msg.transform.rotation.x = 0.0;
        alias_transform_msg.transform.rotation.y = 0.0;
        alias_transform_msg.transform.rotation.z = 0.0;
        alias_transform_msg.transform.rotation.w = 1.0;
        alias_transform_msg.header.frame_id = child_frame_;
        alias_transform_msg.child_frame_id = "base_link";
        br.sendTransform(alias_transform_msg);
    }

        // static tf2_ros::StaticTransformBroadcaster br;
        // geometry_msgs::TransformStamped tf_gt_kiss;
        // tf_gt_kiss.header.stamp = ros::Time::now();
        // tf_gt_kiss.transform.translation.x = 0.0;
        // tf_gt_kiss.transform.translation.y = 0.0;
        // tf_gt_kiss.transform.translation.z = 0.0;
        // tf_gt_kiss.transform.rotation.x = 0.0;
        // tf_gt_kiss.transform.rotation.y = 0.0;
        // tf_gt_kiss.transform.rotation.z = 0.721382357437;
        // tf_gt_kiss.transform.rotation.w = -0.692536998563;
        // tf_gt_kiss.header.frame_id = "base_link_gt";
        // tf_gt_kiss.child_frame_id = "gt_kiss";
        // br.sendTransform(tf_gt_kiss);



    // publish odometry msg
    ROS_INFO("KISS-ICP ROS 1 Odometry Node Initialized");
}

// bool OdometryServer::odomSericeCallback(Odom_service::Request& req, Odom_service::Response& res) { //m
//     nav_msgs::Odometry msg = req.odom;

//     if (seq == 0) return;
// // void OdometryServer::RegisterOdometry(const nav_msgs::Odometry &msg) {

//     // store pose and twist
//     Eigen::Vector3d translation(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
//     Eigen::Quaterniond rotation(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    
//     // odometry_.gtsam_initial_pose = Sophus::SE3d(rotation, translation);
//     odometry_.gtsam_odom_translation = translation;
//     odometry_.gtsam_odom_rotation = rotation;

//     int seq_odom = msg.header.seq;
//     const auto points = utils::PointCloud2ToEigen(msg);
//     const auto timestamps = [&]() -> std::vector<double> {
//         if (!config_.deskew) return {};
//         return utils::GetTimestamps(msg);
//     }();

//     // Register frame, main entry point to KISS-ICP pipeline
//     const auto &[frame, keypoints] = odometry_.RegisterFrame(points, timestamps);

//     // PublishPose
//     const auto pose = odometry_.poses().back();

//     // Convert from Eigen to ROS types
//     const Eigen::Vector3d t_current = pose.translation();
//     const Eigen::Quaterniond q_current = pose.unit_quaternion();

//     // Broadcast the tf
//     geometry_msgs::TransformStamped transform_msg;
//     transform_msg.header.stamp = msg.header.stamp;
//     transform_msg.header.frame_id = odom_frame_;
//     transform_msg.child_frame_id = child_frame_;
//     transform_msg.transform.rotation.x = q_current.x();
//     transform_msg.transform.rotation.y = q_current.y();
//     transform_msg.transform.rotation.z = q_current.z();
//     transform_msg.transform.rotation.w = q_current.w();
//     transform_msg.transform.translation.x = t_current.x();
//     transform_msg.transform.translation.y = t_current.y();
//     transform_msg.transform.translation.z = t_current.z();
//     tf_broadcaster_.sendTransform(transform_msg);

//     // publish odometry msg
//     nav_msgs::Odometry odom_msg;
//     odom_msg.header.stamp = msg.header.stamp;
//     odom_msg.header.frame_id = odom_frame_;
//     odom_msg.child_frame_id = child_frame_;
//     odom_msg.pose.pose.orientation.x = q_current.x();
//     odom_msg.pose.pose.orientation.y = q_current.y();
//     odom_msg.pose.pose.orientation.z = q_current.z();
//     odom_msg.pose.pose.orientation.w = q_current.w();
//     odom_msg.pose.pose.position.x = t_current.x();
//     odom_msg.pose.pose.position.y = t_current.y();
//     odom_msg.pose.pose.position.z = t_current.z();
//     odom_publisher_.publish(odom_msg);

//     nav_msgs::Odometry odom_kiss;
//     odom_kiss.header.stamp = msg.header.stamp;
//     odom_kiss.header.frame_id = odom_frame_;
//     odom_kiss.child_frame_id = child_frame_;

//     Sophus::SE3d second_last_pose;

//     if (odometry_.poses().size() < 2) {
//         second_last_pose = Sophus::SE3d();
//     }
//     else {
//         second_last_pose = odometry_.poses()[odometry_.poses().size()-2];
//     }


//     Sophus::SE3d delta_pose = second_last_pose.inverse() * pose;
    
//     odom_kiss.pose.pose.orientation.x = delta_pose.unit_quaternion().x();
//     odom_kiss.pose.pose.orientation.y = delta_pose.unit_quaternion().y();
//     odom_kiss.pose.pose.orientation.z = delta_pose.unit_quaternion().z();
//     odom_kiss.pose.pose.orientation.w = delta_pose.unit_quaternion().w();
//     odom_kiss.pose.pose.position.x = delta_pose.translation().x();
//     odom_kiss.pose.pose.position.y = delta_pose.translation().y();
//     odom_kiss.pose.pose.position.z = delta_pose.translation().z();
//     // odom_kiss_publisher_.publish(odom_kiss); //m

//     // publish trajectory msg
//     geometry_msgs::PoseStamped pose_msg;
//     pose_msg.pose = odom_msg.pose.pose;
//     pose_msg.header = odom_msg.header;
//     path_msg_.poses.push_back(pose_msg);
//     traj_publisher_.publish(path_msg_);

//     // Publish KISS-ICP internal data, just for debugging
//     std_msgs::Header frame_header = msg.header;
//     frame_header.frame_id = child_frame_;
//     frame_publisher_.publish(utils::EigenToPointCloud2(frame, frame_header));
//     kpoints_publisher_.publish(utils::EigenToPointCloud2(keypoints, frame_header));

//     // Map is referenced to the odometry_frame
//     std_msgs::Header local_map_header = msg.header;
//     local_map_header.frame_id = odom_frame_;
//     local_map_publisher_.publish(utils::EigenToPointCloud2(odometry_.LocalMap(), local_map_header));



//     res.odom = odom_kiss;
//     res.success = true;

//     // odometry_.gtsam_odom_translation = Eigen::Vector3d(req.x, req.y, req.z);
//     // odometry_.gtsam_odom_rotation = Eigen::Vector3d(req.roll, req.pitch, req.yaw);

//     ROS_INFO("Received a service call with a custom odometry message.");
//     return true;
// }




void OdometryServer::RegisterOdometry(const nav_msgs::Odometry &msg) {
    // std::cout << "odom for kiss" << std::endl;
    if (seq == 0) return;
// void OdometryServer::RegisterOdometry(const nav_msgs::Odometry &msg) {

    // store pose and twist
    Eigen::Vector3d translation(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    Eigen::Quaterniond rotation(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    
    // odometry_.gtsam_initial_pose = Sophus::SE3d(rotation, translation);
    odometry_.gtsam_odom_translation = translation;
    odometry_.gtsam_odom_rotation = rotation;

    int seq_odom = msg.header.seq;

    RegisterFrame2(lidar_msg);

    // if (seq_odom == seq) {
    //     RegisterFrame2(lidar_msg);
    // }
    // else {
    //     std::cout << "seq do not match " << seq_odom << " " << seq << std::endl;
    // }

    // if (odometry_.gtsam_t0 == 0) {
    //     odometry_.gtsam_t0 = msg.header.stamp.toSec(); // todo
    //     odometry_.gtsam_current_msg_time = 0; // todo
    //     odometry_.gtsam_last_msg_time = odometry_.gtsam_current_msg_time;
    // } else {
    //     odometry_.gtsam_current_msg_time = msg.header.stamp.toSec(); // todo
    // }
}


void OdometryServer::RegisterFrame(const sensor_msgs::PointCloud2 &msg) {
    seq = msg.header.seq;
    lidar_msg = msg;
    // std::cout << "seq " << seq << std::endl;
}

void OdometryServer::RegisterFrame2(const sensor_msgs::PointCloud2 &msg) {
    // https://github.com/ethz-asl/lidar_undistortion/blob/master/lidar_undistortion/src/lidar_undistorter.cpp

    // deskew here... start

    std::string lidar_frame_id_ = "lidar_link";
    std::string fixed_frame_id_ = "odom_gtsam";

    const auto timestamps = [&]() -> std::vector<double> {
        // if (!config_.deskew) return {};
        return utils::GetTimestamps(msg);
    }();

    pcl::PointCloud<pcl::PointXYZ> pointcloud;

    if (!first_lidar && config_.deskew) {


        pcl::fromROSMsg(msg, pointcloud);

        // Assert that the pointcloud is not empty
        if (pointcloud.empty()) return;

        // Get the start and end times of the pointcloud
        // ros::Time t_start = msg.header.stamp +
        //                     ros::Duration(pointcloud.points.begin()->t * 1e-9);
        // ros::Time t_end = msg.header.stamp +
        //                     ros::Duration((--pointcloud.points.end())->t * 1e-9);

        ros::Time t_start = ros::Time(timestamps.front());
        ros::Time t_end = ros::Time(timestamps.back());

        // TODO: doesnt work because I'm publishing timu tf offline...
        // try {
            // Wait for all transforms to become available
        // if (!waitForTransform(lidar_frame_id_, fixed_frame_id_, t_end, 0.05, 0.25)) {
        // ROS_WARN(
        //     "Could not get correction transform within allotted time. "
        //     "Skipping pointcloud.");
        // return;
        // }
        if (tf_buffer_.canTransform(fixed_frame_id_, lidar_frame_id_, t_start)){
            geometry_msgs::TransformStamped msg_T_F_S_original = tf_buffer_.lookupTransform(fixed_frame_id_, lidar_frame_id_, t_start);
            Eigen::Affine3f T_F_S_original;
            transformMsgToEigen(msg_T_F_S_original.transform, T_F_S_original);

            // Compute the transform used to project the corrected pointcloud back into
            // lidar's scan frame, for more info see the current class' header
            Eigen::Affine3f T_S_F_original = T_F_S_original.inverse();

            // Correct the distortion on all points, using the LiDAR's true pose at
            // each point's timestamp
            uint32_t last_transform_update_t = 0;
            Eigen::Affine3f T_S_original__S_corrected = Eigen::Affine3f::Identity();

            int i = 0;
            geometry_msgs::TransformStamped msg_T_F_S_correct;
            for (auto &point : pointcloud.points) {
            // Check if the current point's timestamp differs from the previous one
            // If so, lookup the new corresponding transform
                ros::Time t_point = ros::Time(timestamps[i]);
                i++;
                
                if (tf_buffer_.canTransform(fixed_frame_id_, lidar_frame_id_, t_point)){
                    msg_T_F_S_correct = tf_buffer_.lookupTransform(fixed_frame_id_, lidar_frame_id_, t_point);
                }

                Eigen::Affine3f T_F_S_correct;
                transformMsgToEigen(msg_T_F_S_correct.transform, T_F_S_correct);
                T_S_original__S_corrected = T_S_F_original * T_F_S_correct;
            
                point = pcl::transformPoint(point, T_S_original__S_corrected);
            
            //   if (point.t != last_transform_update_t) {
            //     last_transform_update_t = point.t;
            //     ros::Time point_t = pointcloud_msg.header.stamp + ros::Duration(0, point.t);
            //     geometry_msgs::TransformStamped msg_T_F_S_correct = tf_buffer_.lookupTransform(fixed_frame_id_, lidar_frame_id_, point_t);
            //     Eigen::Affine3f T_F_S_correct;
            //     transformMsgToEigen(msg_T_F_S_correct.transform, T_F_S_correct);
            //     T_S_original__S_corrected = T_S_F_original * T_F_S_correct;
            //   }

            // Correct the point's distortion, by transforming it into the fixed
            // frame based on the LiDAR sensor's current true pose, and then transform
            // it back into the lidar scan frame
            }
        }

    } else if (config_.deskew && tf_buffer_.canTransform(fixed_frame_id_, lidar_frame_id_, ros::Time(timestamps[0]))) {
        first_lidar = false;
    }

    // end

    const auto x = (first_lidar) ? utils::PointCloud2ToEigen(msg) : utils::PCLToEigen(pointcloud);


    const auto points = utils::PointCloud2ToEigen(msg);

    // Register frame, main entry point to KISS-ICP pipeline
    const auto &[frame, keypoints] = odometry_.RegisterFrame(points, timestamps);

    // PublishPose
    const auto pose = odometry_.poses().back();

    // Convert from Eigen to ROS types
    const Eigen::Vector3d t_current = pose.translation();
    const Eigen::Quaterniond q_current = pose.unit_quaternion();

    // Broadcast the tf
    geometry_msgs::TransformStamped transform_msg;
    transform_msg.header.stamp = msg.header.stamp;
    transform_msg.header.frame_id = odom_frame_;
    transform_msg.child_frame_id = child_frame_;
    transform_msg.transform.rotation.x = q_current.x();
    transform_msg.transform.rotation.y = q_current.y();
    transform_msg.transform.rotation.z = q_current.z();
    transform_msg.transform.rotation.w = q_current.w();
    transform_msg.transform.translation.x = t_current.x();
    transform_msg.transform.translation.y = t_current.y();
    transform_msg.transform.translation.z = t_current.z();
    tf_broadcaster_.sendTransform(transform_msg);

    // publish odometry msg
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = msg.header.stamp;
    odom_msg.header.frame_id = odom_frame_;
    odom_msg.child_frame_id = child_frame_;
    odom_msg.pose.pose.orientation.x = q_current.x();
    odom_msg.pose.pose.orientation.y = q_current.y();
    odom_msg.pose.pose.orientation.z = q_current.z();
    odom_msg.pose.pose.orientation.w = q_current.w();
    odom_msg.pose.pose.position.x = t_current.x();
    odom_msg.pose.pose.position.y = t_current.y();
    odom_msg.pose.pose.position.z = t_current.z();
    odom_publisher_.publish(odom_msg);

    nav_msgs::Odometry odom_kiss;
    odom_kiss.header.stamp = msg.header.stamp;
    odom_kiss.header.frame_id = odom_frame_;
    odom_kiss.child_frame_id = child_frame_;

    Sophus::SE3d second_last_pose;


    if (odometry_.poses().size() < 2) {
        second_last_pose = Sophus::SE3d();
    }
    else {
        second_last_pose = odometry_.poses()[odometry_.poses().size()-2]; // every diff
    }


    // if (odometry_.poses().size() < 11) {
    //     if (odometry_.poses().size() < 2) {
    //         second_last_pose = Sophus::SE3d();
    //     }
    //     else {
    //         second_last_pose = odometry_.poses()[odometry_.poses().size()-2]; // every diff
    //     }
    // }
    // else {
    //     second_last_pose = odometry_.poses()[odometry_.poses().size()-11]; // 10 diff
    // }
    /////


    Sophus::SE3d delta_pose = second_last_pose.inverse() * pose;
    
    odom_kiss.pose.pose.orientation.x = delta_pose.unit_quaternion().x();
    odom_kiss.pose.pose.orientation.y = delta_pose.unit_quaternion().y();
    odom_kiss.pose.pose.orientation.z = delta_pose.unit_quaternion().z();
    odom_kiss.pose.pose.orientation.w = delta_pose.unit_quaternion().w();
    odom_kiss.pose.pose.position.x = delta_pose.translation().x();
    odom_kiss.pose.pose.position.y = delta_pose.translation().y();
    odom_kiss.pose.pose.position.z = delta_pose.translation().z();
    // std::cout << "TEST " << kiss_icp::test << std::endl;
    odom_kiss.pose.covariance[0] = odometry_.degenerate;
    odometry_.degenerate = false;
    
    odom_kiss_publisher_.publish(odom_kiss);
    // std::cout << "delta_tr_kiss: " << odom_kiss.pose.pose.position.x << " " << odom_kiss.pose.pose.position.y << " " << odom_kiss.pose.pose.position.z << std::endl;

    // publish trajectory msg
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.pose = odom_msg.pose.pose;
    pose_msg.header = odom_msg.header;
    path_msg_.poses.push_back(pose_msg);
    traj_publisher_.publish(path_msg_);

    // Publish KISS-ICP internal data, just for debugging
    std_msgs::Header frame_header = msg.header;
    // frame_header.frame_id = "gt_kiss";
    frame_header.frame_id = child_frame_;
    frame_publisher_.publish(utils::EigenToPointCloud2(frame, frame_header));
    kpoints_publisher_.publish(utils::EigenToPointCloud2(keypoints, frame_header));

    // Map is referenced to the odometry_frame
    std_msgs::Header local_map_header = msg.header;
    local_map_header.frame_id = odom_frame_;
    local_map_publisher_.publish(utils::EigenToPointCloud2(odometry_.LocalMap(), local_map_header));
}


void OdometryServer::save_path() {
        
    std::ofstream myfile;
    // myfile.open(ros::package::getPath("kiss-icp") + "/data/kiss_trajectory.txt");

    std::string output_file;
    ros::param::get("~output_file", output_file);
    myfile.open("/home/marco/mt_rlio/catkin_ws_kiss/src/kiss-icp/data/" + output_file);

    Eigen::Quaterniond R_base_lidar(-0.692536998563, 0, 0, 0.721382357437);

    std::cout << "nr of poses to write: " << path_msg_.poses.size() << std::endl;

    for (int i = 0; i < path_msg_.poses.size(); i++) {

        Eigen::Vector3d position(path_msg_.poses[i].pose.position.x, path_msg_.poses[i].pose.position.y, path_msg_.poses[i].pose.position.z);

        Eigen::Quaterniond rotation(path_msg_.poses[i].pose.orientation.w, path_msg_.poses[i].pose.orientation.x, path_msg_.poses[i].pose.orientation.y, path_msg_.poses[i].pose.orientation.z);

        position = R_base_lidar * position;
        rotation = R_base_lidar * rotation;

        myfile << std::setprecision(15) << path_msg_.poses[i].header.stamp.toSec() << " "
                << position.x() << " " << position.y() << " " << position.z() << " "
                << rotation.x() << " " << rotation.y() << " " << rotation.z() << " " << rotation.w() << "\n";

        // myfile << setprecision(15) << timestamp_vector[i] << " " << pose.translation().x() << " " << pose.translation().y() << " " << pose.translation().z() << " " << pose.rotation().toQuaternion().x() << " " << pose.rotation().toQuaternion().y() << " " << pose.rotation().toQuaternion().z() << " " << pose.rotation().toQuaternion().w() << endl;
    }
    myfile.close();
}




}  // namespace kiss_icp_ros

int main(int argc, char **argv) {
    ros::init(argc, argv, "kiss_icp");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    kiss_icp_ros::OdometryServer node(nh, nh_private);

    while (ros::ok()) {
        ros::spinOnce();
    }

    node.save_path();

    return 0;
}
