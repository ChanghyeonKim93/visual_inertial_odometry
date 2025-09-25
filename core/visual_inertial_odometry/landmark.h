#ifndef VISUAL_INERTIAL_ODOMETRY_LANDMARK_H_
#define VISUAL_INERTIAL_ODOMETRY_LANDMARK_H_

#include <memory>

#include "Eigen/Dense"

#include "visual_inertial_odometry/frame.h"
#include "visual_inertial_odometry/types.h"

namespace visual_inertial_odometry {

class Frame;

using FramePtr = std::shared_ptr<Frame>;

class Landmark {
 public:
  struct State {
    bool is_alive{true};
    bool is_triangulated{false};
    bool is_bundled{false};
  };

  Landmark() {}

  ~Landmark() {}

  void AddFrame(const FramePtr& frame) { related_frames_.push_back(frame); }

 private:
  int id_{-1};                 // feature unique id
  std::vector<Vec2f> pixels_;  // 2D pixel history of this landmark
  std::vector<FramePtr> related_frames_;

  State state_;

  int age_{0};  // tracking age

  inline static int landmark_counter_{0};
};

using LandmarkPtr = std::shared_ptr<Landmark>;

}  // namespace visual_inertial_odometry

#endif  // VISUAL_INERTIAL_ODOMETRY_LANDMARK_H_