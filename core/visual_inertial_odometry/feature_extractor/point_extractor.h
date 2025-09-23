#ifndef VISUAL_INERTIAL_ODOMETRY_FEATURE_EXTRACTOR_POINT_EXTRACTOR_H_
#define VISUAL_INERTIAL_ODOMETRY_FEATURE_EXTRACTOR_POINT_EXTRACTOR_H_

#include "visual_inertial_odometry/image.h"
#include "visual_inertial_odometry/types.h"

namespace visual_inertial_odometry {
namespace feature_extractor {

class PointExtractor {
 public:
  PointExtractor() {}

  ~PointExtractor() {}

  virtual void Extract(const Image& image,
                       std::vector<PointFeature>* point_feature_list) = 0;
};

}  // namespace feature_extractor
}  // namespace visual_inertial_odometry

#endif  // VISUAL_INERTIAL_ODOMETRY_FEATURE_EXTRACTOR_POINT_EXTRACTOR_H_
