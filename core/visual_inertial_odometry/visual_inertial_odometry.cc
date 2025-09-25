#include "visual_inertial_odometry/visual_inertial_odometry.h"

#include <iostream>
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

void VisualInertialOdometry::TrackImage(const double timestamp,
                                        const Image& input_image) {
  (void)timestamp;
  // TODO(@): implement this function
  const Image image = ApplyHistogramEqualization(input_image);

  if (prev_pixels_.empty()) {
    point_extractor_->InitializeFeatureOccupancyGrid(
        image.cols, image.rows, parameters_.feature.grid_size,
        &feature_occupancy_grid_);
    const auto curr_pixels =
        point_extractor_->ExtractWithBucketing(image, feature_occupancy_grid_);
    prev_pixels_ = std::move(curr_pixels);
    prev_image_ = image;
    return;
  }

  // Track previous features
  std::vector<Vec2f> tracked_pixels = prev_pixels_;
  point_tracker_->Track(prev_image_, image, prev_pixels_, &tracked_pixels);

  std::vector<Vec2f> survived_pixels;
  survived_pixels.reserve(tracked_pixels.size());
  for (auto it = tracked_pixels.begin(); it != tracked_pixels.end(); ++it) {
    if (it->x() == -1.0f) continue;
    survived_pixels.push_back(*it);
  }
  point_extractor_->UpdateFeatureOccupancyGrid(survived_pixels,
                                               &feature_occupancy_grid_);
  const auto new_pixels =
      point_extractor_->ExtractWithBucketing(image, feature_occupancy_grid_);

  for (const auto& p : new_pixels) survived_pixels.push_back(p);

  // Draw extracted features for debugging
  debug_image_ = image.clone();
  debug_image_.convertTo(debug_image_, CV_8UC3);
  for (const auto& p : survived_pixels) {
    cv::circle(debug_image_, cv::Point2f(p.x(), p.y()), 3,
               cv::Scalar(255, 0, 255), -1);
  }
  prev_pixels_ = survived_pixels;
  prev_image_ = image;

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
  clahe->setClipLimit(3.0);
  clahe->apply(image, equalized_image);
  return equalized_image;
}

}  // namespace visual_inertial_odometry