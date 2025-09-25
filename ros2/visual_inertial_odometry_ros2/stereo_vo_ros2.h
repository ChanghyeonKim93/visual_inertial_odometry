#ifndef VISUAL_INERTIAL_ODOMETRY_ROS2_VISUAL_INERTIAL_ODOMETRY_ROS2_H_
#define VISUAL_INERTIAL_ODOMETRY_ROS2_VISUAL_INERTIAL_ODOMETRY_ROS2_H_

#include <algorithm>
#include <cstring>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>

#include "Eigen/Dense"

#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "visual_inertial_odometry/visual_inertial_odometry.h"

using ImageMsg = sensor_msgs::msg::Image;
using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
using OdometryMsg = nav_msgs::msg::Odometry;
using ImuMsg = sensor_msgs::msg::Imu;

class VisualInertialOdometryNode : public rclcpp::Node {
 public:
  VisualInertialOdometryNode(const std::string& node_name) {}

  ~VisualInertialOdometryNode() {}

 private:
  void CallbackImage(const ImageMsg::ConstSharedPtr& msg);

  rclcpp::Subscription<ImageMsg>::SharedPtr sub_image_;

  rclcpp::Publisher<OdometryMsg>::SharedPtr pub_pose_;
  rclcpp::Publisher<PathMsg>::SharedPtr pub_trajectory_;
  PathMsg msg_trajectory_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_image_;
  std::string topicname_debug_image_;
  ImageMsg msg_debug_image_;

  std::unique_ptr<StereoVO> stereo_vo_;
};

#endif
