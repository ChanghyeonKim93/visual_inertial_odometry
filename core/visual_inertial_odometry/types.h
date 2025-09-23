#ifndef VISUAL_INERTIAL_ODOMETRY_TYPES_H_
#define VISUAL_INERTIAL_ODOMETRY_TYPES_H_

#include "Eigen/Dense"

namespace visual_inertial_odometry {

using Vec2f = Eigen::Vector2f;
using Vec3f = Eigen::Vector3f;
using Pose3f = Eigen::Isometry3f;

using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;
using Pose3d = Eigen::Isometry3d;

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

struct PointFeature {
  uint64_t id{0};
  Vec2f pixel{Vec2f::Zero()};
  float response{0.0f};
  float scale{0.0f};
  float angle{0.0f};
  int octave{0};
  // Eigen::Matrix<float, 32, 1> descriptor{Eigen::Matrix<float, 32,
  // 1>::Zero()};
};

struct ImuData {
  double timestamp{0.0};
  Vec3d linear_acceleration{Vec3d::Zero()};
  Vec3d angular_velocity{Vec3d::Zero()};
};

}  // namespace visual_inertial_odometry

#endif  // VISUAL_INERTIAL_ODOMETRY_TYPES_H_