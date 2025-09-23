#ifndef VISUAL_INERTIAL_ODOMETRY_FEATURE_TRACKER_POINT_TRACKER_H_
#define VISUAL_INERTIAL_ODOMETRY_FEATURE_TRACKER_POINT_TRACKER_H_

#include "visual_inertial_odometry/image.h"
#include "visual_inertial_odometry/types.h"

namespace visual_inertial_odometry {
namespace feature_tracker {

class PointTracker {
 public:
  PointTracker() {}

  ~PointTracker() {}

  virtual bool Track(const Image& src, const Image& dst,
                     const std::vector<PointFeature>& src_pixel_list,
                     std::vector<PointFeature>* dst_pixel_list) = 0;
};

}  // namespace feature_tracker
}  // namespace visual_inertial_odometry

#endif  // VISUAL_INERTIAL_ODOMETRY_FEATURE_TRACKER_POINT_TRACKER_H_