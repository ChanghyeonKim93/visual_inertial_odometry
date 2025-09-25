#ifndef VISUAL_INERTIAL_ODOMETRY_CAMERA_H_
#define VISUAL_INERTIAL_ODOMETRY_CAMERA_H_

#include <memory>
#include <string>

#include "visual_inertial_odometry/types.h"

namespace visual_inertial_odometry {

class Camera {
 public:
  Camera() {}

  ~Camera() {}

  Vec2f ProjectPoint(const Vec3f& point) const {
    // TODO(@): implement this function
    (void)point;
    return Vec2f::Zero();
  };

  Vec3f GetBearingVector(const Vec2f& pixel) const {
    // TODO(@): implement this function.
    (void)pixel;
    return Vec3f::Zero();
  };

  void UndistortPoints(const std::vector<Vec2f>& distorted_points,
                       std::vector<Vec2f>* undistorted_points) const {
    // TODO(@): implement this function
    (void)distorted_points;
    undistorted_points->clear();
  };

 private:
  std::string name_{""};
};

using CameraPtr = std::shared_ptr<Camera>;

}  // namespace visual_inertial_odometry

#endif  // VISUAL_INERTIAL_ODOMETRY_CAMERA_H_
