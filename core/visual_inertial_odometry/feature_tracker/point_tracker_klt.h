#ifndef VISUAL_INERTIAL_ODOMETRY_FEATURE_TRACKER_POINT_TRACKER_KLT_H_
#define VISUAL_INERTIAL_ODOMETRY_FEATURE_TRACKER_POINT_TRACKER_KLT_H_

#include "visual_inertial_odometry/feature_tracker/point_tracker.h"

namespace visual_inertial_odometry {
namespace feature_tracker {

class PointTrackerKLT : public PointTracker {
 public:
  PointTrackerKLT() {}

  ~PointTrackerKLT() {}

  bool Track(const Image& src, const Image& dst,
             const std::vector<PointFeature>& src_pixel_list,
             std::vector<PointFeature>* dst_pixel_list) final;
};

}  // namespace feature_tracker
}  // namespace visual_inertial_odometry

#endif  // VISUAL_INERTIAL_ODOMETRY_FEATURE_TRACKER_POINT_TRACKER_KLT_H_