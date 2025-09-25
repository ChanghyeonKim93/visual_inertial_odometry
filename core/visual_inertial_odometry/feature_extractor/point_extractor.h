#ifndef VISUAL_INERTIAL_ODOMETRY_FEATURE_EXTRACTOR_POINT_EXTRACTOR_H_
#define VISUAL_INERTIAL_ODOMETRY_FEATURE_EXTRACTOR_POINT_EXTRACTOR_H_

#include <memory>
#include <vector>

#include "visual_inertial_odometry/types.h"

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

namespace visual_inertial_odometry {
namespace feature_extractor {

class PointExtractor {
 public:
  PointExtractor();

  ~PointExtractor();

  void InitializeFeatureOccupancyGrid(
      const int image_width, const int image_height, const int grid_size,
      FeatureOccupancyGrid* feature_occupancy_grid);

  void UpdateFeatureOccupancyGrid(const std::vector<Vec2f>& pixels,
                                  FeatureOccupancyGrid* feature_occupancy_grid);

  void ResetFeatureOccupancyGrid(FeatureOccupancyGrid* feature_occupancy_grid);

  std::vector<Vec2f> ExtractWithBucketing(
      const Image& image, const FeatureOccupancyGrid& feature_occupancy_grid);

 private:
  cv::Ptr<cv::ORB> orb_detector_;
};

}  // namespace feature_extractor
}  // namespace visual_inertial_odometry

#endif  // VISUAL_INERTIAL_ODOMETRY_FEATURE_EXTRACTOR_POINT_EXTRACTOR_H_
