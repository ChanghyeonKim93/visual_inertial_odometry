#include "visual_inertial_odometry/visual_inertial_odometry.h"

namespace visual_inertial_odometry {

VisualInertialOdometry::VisualInertialOdometry(const Parameters& parameters)
    : parameters_(parameters) {
  point_tracker_ = std::make_unique<feature_tracker::PointTrackerKLT>();
  point_extractor_ = std::make_unique<feature_extractor::PointExtractor>();
}

VisualInertialOdometry::~VisualInertialOdometry() {}

void VisualInertialOdometry::IntegrateImuPreintegrator(
    const ImuData& imu_data) {
  // TODO(@): implement IMU preintegration
  // TODO(@): update `current_pose_` using IMU preintegration
  (void)current_pose_;
}

void VisualInertialOdometry::TrackImage(const Image& input_image) {
  // TODO(@): implement this function
  const Image image = ApplyHistogramEqualization(input_image);

  if (previous_frame_ == nullptr) {
    previous_frame_ = CreateInitialFrame(image);
    return;
  }

  // Track previous features

  // Get Prior pose from IMU preintegrator

  // Estimate Pose (single pose estimation)

  // Decide if need new keyframe
}

PoseStamped VisualInertialOdometry::GetCurrentPose() const {
  return current_pose_;
}

FramePtr VisualInertialOdometry::CreateInitialFrame(const Image& image) {
  const auto extracted_features = ExtractFeatures(image);
  const auto distributed_features =
      DistributeFeatures(extracted_features, parameters_.feature.grid_size);
  FramePtr new_frame = std::make_shared<Frame>();

  return new_frame;
}

}  // namespace visual_inertial_odometry