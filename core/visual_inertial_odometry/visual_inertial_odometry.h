#ifndef VISUAL_INERTIAL_ODOMETRY_VISUAL_INERTIAL_ODOMETRY_H_
#define VISUAL_INERTIAL_ODOMETRY_VISUAL_INERTIAL_ODOMETRY_H_

#include <deque>
#include <memory>
#include <unordered_set>

#include "visual_inertial_odometry/feature_extractor/point_extractor.h"
#include "visual_inertial_odometry/feature_tracker/point_tracker.h"
#include "visual_inertial_odometry/frame.h"
#include "visual_inertial_odometry/landmark.h"
#include "visual_inertial_odometry/types.h"

namespace visual_inertial_odometry {

struct Parameters {
  struct {
    int max_features_to_extract{1000};
    int grid_size{40};
    int max_num_features_per_cell{3};
  } feature;
  struct {
    double translation_threshold{0.05};
    double rotation_threshold{0.1};
    int max_num_keyframes{10};
  } keyframe;
};

class VisualInertialOdometry {
 public:
  explicit VisualInertialOdometry(const Parameters& parameters);

  ~VisualInertialOdometry();

  void IntegrateImuPreintegrator(const ImuData& imu_data);

  void TrackImage(const double timestamp, const Image& img);

  PoseStamped GetCurrentPose() const;

  Image GetDebugImage() const;

 private:
  Image ApplyHistogramEqualization(const Image& img);

  const Parameters parameters_;

  std::unordered_set<FramePtr> all_frames_;
  std::unordered_set<LandmarkPtr> all_landmarks_;
  std::deque<FramePtr> keyframes_;  // sliding window of keyframes

  FramePtr previous_frame_{nullptr};

  Image prev_image_;
  std::vector<Vec2f> prev_pixels_;

  FeatureOccupancyGrid feature_occupancy_grid_;
  PoseStamped current_pose_;

  Image debug_image_;

  std::unique_ptr<feature_tracker::PointTracker> point_tracker_{nullptr};
  std::unique_ptr<feature_extractor::PointExtractor> point_extractor_{nullptr};
};

}  // namespace visual_inertial_odometry

#endif  // VISUAL_INERTIAL_ODOMETRY_VISUAL_INERTIAL_ODOMETRY_H_