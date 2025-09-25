#include "visual_inertial_odometry/visual_inertial_odometry.h"

namespace visual_inertial_odometry {

VisualInertialOdometry::VisualInertialOdometry(const Parameters& parameters)
    : parameters_(parameters) {
  point_tracker_ = std::make_unique<feature_tracker::PointTracker>();
  point_extractor_ = std::make_unique<feature_extractor::PointExtractor>();
}

VisualInertialOdometry::~VisualInertialOdometry() {}

void VisualInertialOdometry::IntegrateImuPreintegrator(
    const ImuData& imu_data) {
  // TODO(@): implement IMU preintegration
  // TODO(@): update `current_pose_` using IMU preintegration
  (void)imu_data;
  (void)current_pose_;
}

void VisualInertialOdometry::TrackImage(const Image& input_image) {
  // TODO(@): implement this function
  const Image image = ApplyHistogramEqualization(input_image);

  if (prev_pixels_.empty()) {
    point_extractor_->InitializeFeatureOccupancyGrid(
        image.cols, image.rows, parameters_.feature.grid_size,
        &feature_occupancy_grid_);
    const auto curr_pixels =
        point_extractor_->ExtractWithBucketing(image, feature_occupancy_grid_);
    prev_pixels_ = std::move(curr_pixels);
    return;
  }

  point_extractor_->UpdateFeatureOccupancyGrid(prev_pixels_,
                                               &feature_occupancy_grid_);
  const auto curr_pixels =
      point_extractor_->ExtractWithBucketing(image, feature_occupancy_grid_);

  // Track previous features

  // Get Prior pose from IMU preintegrator

  // Estimate Pose (single pose estimation)

  // Decide if need new keyframe
}

PoseStamped VisualInertialOdometry::GetCurrentPose() const {
  return current_pose_;
}

Image VisualInertialOdometry::GetDebugImage() const { return debug_image_; }

Image VisualInertialOdometry::ApplyHistogramEqualization(const Image& image) {
  // Use CLAHE
  Image equalized_image;
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(4.0);
  clahe->apply(image, equalized_image);
  return equalized_image;
}

}  // namespace visual_inertial_odometry