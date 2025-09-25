#ifndef VISUAL_INERTIAL_ODOMETRY_FEATURE_TRACKER_POINT_TRACKER_H_
#define VISUAL_INERTIAL_ODOMETRY_FEATURE_TRACKER_POINT_TRACKER_H_

#include "visual_inertial_odometry/types.h"

namespace visual_inertial_odometry {
namespace feature_tracker {

class PointTracker {
 public:
  PointTracker();

  ~PointTracker();

  void Track(const Image& image_1, const Image& image_2,
             const std::vector<Vec2f>& pixels_1, std::vector<Vec2f>* pixels_2);
};

}  // namespace feature_tracker
}  // namespace visual_inertial_odometry

#endif  // VISUAL_INERTIAL_ODOMETRY_FEATURE_TRACKER_POINT_TRACKER_H_