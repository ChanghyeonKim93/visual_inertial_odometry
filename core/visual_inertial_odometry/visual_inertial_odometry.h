#ifndef VISUAL_INERTIAL_ODOMETRY_VISUAL_INERTIAL_ODOMETRY_H_
#define VISUAL_INERTIAL_ODOMETRY_VISUAL_INERTIAL_ODOMETRY_H_

#include <deque>
#include <memory>
#include <unordered_set>

#include "visual_inertial_odometry/feature_extractor/point_extractor.h"
#include "visual_inertial_odometry/feature_tracker/point_tracker.h"
#include "visual_inertial_odometry/feature_tracker/point_tracker_klt.h"
#include "visual_inertial_odometry/frame.h"
#include "visual_inertial_odometry/image.h"
#include "visual_inertial_odometry/landmark.h"
#include "visual_inertial_odometry/types.h"

namespace visual_inertial_odometry {

struct Parameters {
  struct {
    int max_features_to_extract{1000};
    int grid_size{30};
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

  void TrackImage(const Image& img);

  PoseStamped GetCurrentPose() const;

 private:
  FramePtr CreateInitialFrame(const Image& img);
  Image ApplyHistogramEqualization(const Image& img);
  std::vector<PointFeature> ExtractFeatures(const Image& img);
  std::vector<PointFeature> DistributeFeatures(
      const std::vector<PointFeature>& features, const int grid_size);
  FramePtr CreateFrame(const Image& img,
                       const std::vector<PointFeature>& features);
  void TrackFeatures(const FramePtr& src_frame, const FramePtr& dst_frame,
                     const std::vector<PointFeature>& src_features,
                     std::vector<PointFeature>* dst_features);
  Pose3d GetPriorPose() const;  // from IMU preintegration
  Pose3d EstimatePose(const FramePtr& src_frame, const FramePtr& dst_frame,
                      const std::vector<PointFeature>& src_features,
                      const std::vector<PointFeature>& dst_features);

  bool NeedNewKeyFrame() const;
  void InsertKeyFrame(const FramePtr& keyframe);

  const Parameters parameters_;

  std::unordered_set<FramePtr> all_frames_;
  std::unordered_set<LandmarkPtr> all_landmarks_;
  std::deque<FramePtr> keyframes_;  // sliding window of keyframes

  FramePtr previous_frame_{nullptr};
  PoseStamped current_pose_;

  std::unique_ptr<feature_tracker::PointTracker> point_tracker_{nullptr};
  std::unique_ptr<feature_extractor::PointExtractor> point_extractor_{nullptr};
};

}  // namespace visual_inertial_odometry

#endif  // VISUAL_INERTIAL_ODOMETRY_VISUAL_INERTIAL_ODOMETRY_H_