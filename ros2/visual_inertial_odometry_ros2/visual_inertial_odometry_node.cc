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
  VisualInertialOdometryNode(const std::string& node_name) {
    vio_ = std::make_unique<VisualInertialOdometry>(
        VisualInertialOdometry::Parameters());
  }

  ~VisualInertialOdometryNode() {}

 private:
  void CallbackImage(const ImageMsg::ConstSharedPtr& msg) {
    if (msg == nullptr) return;
    if (msg->data.empty()) return;
    if (msg->encoding != "mono8" && msg->encoding != "8UC1") {
      RCLCPP_ERROR(this->get_logger(),
                   "CallbackImage: unsupported image encoding: %s",
                   msg->encoding.c_str());
      return;
    }

    // Convert ROS image message to cv::Mat
    cv::Mat image(msg->height, msg->width, CV_8UC1);
    if (msg->step != static_cast<size_t>(image.cols)) {
      RCLCPP_ERROR(this->get_logger(),
                   "CallbackImage: step does not match image width");
      return;
    }
    std::memcpy(image.data, msg->data.data(), msg->data.size());

    // Process the image using VIO
    vio_->TrackImage(image);
  }

  rclcpp::Subscription<ImageMsg>::SharedPtr sub_image_;

  rclcpp::Publisher<OdometryMsg>::SharedPtr pub_pose_;
  rclcpp::Publisher<PathMsg>::SharedPtr pub_trajectory_;
  PathMsg msg_trajectory_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_debug_image_;
  std::string topicname_debug_image_;
  ImageMsg msg_debug_image_;

  std::unique_ptr<VisualInertialOdometry> vio_{nullptr};
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    std::string node_name = "visual_inertial_odometry_node";
    rclcpp::spin(std::make_shared<VisualInertialOdometryNode>(node_name));
    rclcpp::shutdown();
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
  }
  return 0;
}
