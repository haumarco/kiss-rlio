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
#pragma once

// KISS-ICP
#include "kiss_icp/pipeline/KissICP.hpp"

// ROS
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include <tf2_ros/transform_listener.h>


// #include "kiss_icp/Odom_service.h"



namespace kiss_icp_ros {

class OdometryServer {
public:
    /// OdometryServer constructor
    OdometryServer(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);
    void save_path();

    template <typename T>
    void transformMsgToEigen(const geometry_msgs::Transform &transform_msg,
                            T &transform) {  // NOLINT
        transform =
            Eigen::Translation3f(transform_msg.translation.x,
                                transform_msg.translation.y,
                                transform_msg.translation.z) *
            Eigen::Quaternionf(transform_msg.rotation.w, transform_msg.rotation.x,
                            transform_msg.rotation.y, transform_msg.rotation.z);
        }

    bool waitForTransform(const std::string &from_frame_id,
                                            const std::string &to_frame_id,
                                            const ros::Time &frame_timestamp,
                                            const double &sleep_between_retries__s,
                                            const double &timeout__s) {
    // Total time spent waiting for the updated pose
    ros::WallDuration t_waited(0.0);
    // Maximum time to wait before giving up
    ros::WallDuration t_max(timeout__s);
    // Timeout between each update attempt
    const ros::WallDuration t_sleep(sleep_between_retries__s);
    while (t_waited < t_max) {
        if (tf_buffer_.canTransform(to_frame_id, from_frame_id, frame_timestamp)) {
        return true;
        }
        t_sleep.sleep();
        t_waited += t_sleep;
    }
    ROS_WARN("Waited %.3fs, but still could not get the TF from %s to %s",
            t_waited.toSec(), from_frame_id.c_str(), to_frame_id.c_str());
    return false;
    }

private:
    /// Register new frame
    void RegisterFrame(const sensor_msgs::PointCloud2 &msg);
    void RegisterFrame2(const sensor_msgs::PointCloud2 &msg);

    void RegisterOdometry(const nav_msgs::Odometry &msg);

    // bool odomSericeCallback(kiss_icp::srv::Odom_service::Request &req, kiss_icp::srv::Odom_service::Response &res);



    /// Ros node stuff
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    int queue_size_{1};

    /// Tools for broadcasting TFs.
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    /// Data subscribers.
    ros::Subscriber pointcloud_sub_;
    ros::Subscriber odom_sub_;

    /// Data publishers.
    ros::Publisher odom_publisher_;
    ros::Publisher traj_publisher_;
    nav_msgs::Path path_msg_;
    ros::Publisher frame_publisher_;
    ros::Publisher kpoints_publisher_;
    ros::Publisher local_map_publisher_;

    ros::Publisher odom_kiss_publisher_;
    int seq=0;
    sensor_msgs::PointCloud2 lidar_msg;

    ros::ServiceServer odom_serice;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    bool first_lidar = true;


    /// KISS-ICP
    kiss_icp::pipeline::KissICP odometry_;
    kiss_icp::pipeline::KISSConfig config_;

    /// Global/map coordinate frame.
    std::string odom_frame_{"odom"};
    std::string child_frame_{"base_link"};
};

}  // namespace kiss_icp_ros
