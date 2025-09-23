#ifndef VISUAL_INERTIAL_ODOMETRY_FRAME_H_
#define VISUAL_INERTIAL_ODOMETRY_FRAME_H_

#include <memory>

#include "Eigen/Dense"

#include "visual_inertial_odometry/camera.h"
#include "visual_inertial_odometry/landmark.h"
#include "visual_inertial_odometry/types.h"

namespace visual_inertial_odometry {

class Landmark;
using LandmarkPtr = std::shared_ptr<Landmark>;

class Frame {
 public:
  Frame() {}

  ~Frame() {}

  void SetPose(const Pose3d& pose) {
    pose_ = pose;
    inverse_pose_ = pose_.inverse();
  }

  const Pose3d& GetPose() const { return pose_; }

  const Pose3d& GetInversePose() const { return inverse_pose_; }

  void AddLandmark(const LandmarkPtr& landmark) {
    related_landmarks_.emplace_back(landmark);
  }

 private:
  int id_{-1};
  double timestamp_{0.0};
  CameraPtr camera_{nullptr};

  std::vector<LandmarkPtr> related_landmarks_;

  Pose3d pose_{Pose3d::Identity()};
  Pose3d inverse_pose_{Pose3d::Identity()};

  inline static int frame_counter_{0};  // Unique frame counter. (static)
};

using FramePtr = std::shared_ptr<Frame>;

}  // namespace visual_inertial_odometry

#endif  // VISUAL_INERTIAL_ODOMETRY_FRAME_H_