#include "visual_inertial_odometry/feature_tracker/point_tracker.h"

#include "opencv2/video/tracking.hpp"

namespace visual_inertial_odometry {
namespace feature_tracker {

PointTracker::PointTracker() {}

PointTracker::~PointTracker() {}

void PointTracker::Track(const Image& prev_image, const Image& curr_image,
                         const std::vector<Vec2f>& prev_pixels,
                         std::vector<Vec2f>* curr_pixels) {
  if (prev_image.empty() || curr_image.empty() || prev_pixels.empty() ||
      curr_pixels == nullptr) {
    return;
  }

  std::vector<cv::Point2f> prev_corners;
  prev_corners.reserve(prev_pixels.size());
  for (const auto& pixel : prev_pixels)
    prev_corners.emplace_back(pixel.x(), pixel.y());

  std::vector<cv::Point2f> curr_corners;
  curr_corners.reserve(prev_pixels.size());
  // Set prior pixels as initial guess.
  if (curr_pixels->size() == prev_pixels.size()) {
    for (const auto& pixel : *curr_pixels)
      curr_corners.emplace_back(pixel.x(), pixel.y());
  } else {
    curr_corners = prev_corners;
  }

  for (const auto& pixel : prev_pixels)
    prev_corners.emplace_back(pixel.x(), pixel.y());

  std::vector<uchar> tracking_status;
  std::vector<float> tracking_error;
  cv::calcOpticalFlowPyrLK(prev_image, curr_image, prev_corners, curr_corners,
                           tracking_status, tracking_error);

  curr_pixels->clear();
  curr_pixels->resize(prev_pixels.size());
  for (size_t i = 0; i < tracking_status.size(); ++i) {
    auto& curr_pixel = curr_pixels->at(i);
    curr_pixel.x() = -1.0f;
    curr_pixel.y() = -1.0f;

    if (!tracking_status[i]) continue;
    if (tracking_error[i] > 20.0f) continue;

    curr_pixel.x() = curr_corners[i].x;
    curr_pixel.y() = curr_corners[i].y;
  }
}

}  // namespace feature_tracker
}  // namespace visual_inertial_odometry
