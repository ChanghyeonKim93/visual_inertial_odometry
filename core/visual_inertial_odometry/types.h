#ifndef VISUAL_INERTIAL_ODOMETRY_TYPES_H_
#define VISUAL_INERTIAL_ODOMETRY_TYPES_H_

#include <set>
#include <vector>

#include "Eigen/Dense"
#include "opencv2/core.hpp"

namespace visual_inertial_odometry {

using Vec2f = Eigen::Vector2f;
using Vec3f = Eigen::Vector3f;
using Pose3f = Eigen::Isometry3f;

using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;
using Pose3d = Eigen::Isometry3d;

using Image = cv::Mat;

struct PoseStamped {
  double timestamp{0.0};
  Pose3d pose{Pose3d::Identity()};
};

struct ImageSize {
  int width{0};
  int height{0};
};

struct Intrinsics {
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
};

struct ImuData {
  double timestamp{0.0};
  Vec3d linear_acceleration{Vec3d::Zero()};
  Vec3d angular_velocity{Vec3d::Zero()};
};

struct FeatureOccupancyGrid {
  int image_width{0};
  int image_height{0};
  int grid_size{0};
  int num_grid_along_width{0};
  int num_grid_along_height{0};
  std::vector<bool> is_occupied_list;
};

}  // namespace visual_inertial_odometry

#endif  // VISUAL_INERTIAL_ODOMETRY_TYPES_H_