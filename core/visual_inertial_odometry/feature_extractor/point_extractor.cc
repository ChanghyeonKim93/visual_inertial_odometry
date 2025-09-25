#include "visual_inertial_odometry/feature_extractor/point_extractor.h"

#include <iostream>

#include "opencv2/imgproc.hpp"

namespace visual_inertial_odometry {
namespace feature_extractor {

PointExtractor::PointExtractor() {
  orb_detector_ = cv::ORB::create();
  orb_detector_->setMaxFeatures(10000);
  orb_detector_->setScaleFactor(1.05);
  orb_detector_->setNLevels(1);
  orb_detector_->setEdgeThreshold(31);
  orb_detector_->setFirstLevel(0);
  orb_detector_->setWTA_K(2);
  orb_detector_->setScoreType(cv::ORB::HARRIS_SCORE);
  orb_detector_->setPatchSize(31);
  orb_detector_->setFastThreshold(15);
}

PointExtractor::~PointExtractor() {}

void PointExtractor::InitializeFeatureOccupancyGrid(
    const int image_width, const int image_height, const int grid_size,
    FeatureOccupancyGrid* feature_occupancy_grid) {
  if (feature_occupancy_grid == nullptr)
    throw std::runtime_error(
        "PointExtractor::InitializeFeatureOccupancyGrid: input pointer is "
        "nullptr");

  feature_occupancy_grid->image_width = image_width;
  feature_occupancy_grid->image_height = image_height;
  feature_occupancy_grid->grid_size = grid_size;
  feature_occupancy_grid->num_grid_along_width = static_cast<int>(
      std::floor(image_width / static_cast<double>(grid_size)));
  feature_occupancy_grid->num_grid_along_height = static_cast<int>(
      std::floor(image_height / static_cast<double>(grid_size)));
  feature_occupancy_grid->is_occupied_list.resize(
      feature_occupancy_grid->num_grid_along_width *
          feature_occupancy_grid->num_grid_along_height,
      false);
}

void PointExtractor::UpdateFeatureOccupancyGrid(
    const std::vector<Vec2f>& pixels,
    FeatureOccupancyGrid* feature_occupancy_grid) {
  if (feature_occupancy_grid == nullptr)
    throw std::runtime_error(
        "PointExtractor::InitializeFeatureOccupancyGrid: input pointer is "
        "nullptr");

  const double inverse_grid_size = 1.0 / feature_occupancy_grid->grid_size;
  const int num_grid_along_width = feature_occupancy_grid->num_grid_along_width;
  const int num_grid_along_height =
      feature_occupancy_grid->num_grid_along_height;

  auto& occupancy_grid = feature_occupancy_grid->is_occupied_list;
  std::fill(occupancy_grid.begin(), occupancy_grid.end(), false);

  for (const auto& pixel : pixels) {
    const int x_index =
        static_cast<int>(std::floor(pixel.x() * inverse_grid_size));
    const int y_index =
        static_cast<int>(std::floor(pixel.y() * inverse_grid_size));

    if (x_index < 0 || x_index >= num_grid_along_width || y_index < 0 ||
        y_index >= num_grid_along_height)
      continue;

    const int grid_index = y_index + x_index * num_grid_along_height;
    occupancy_grid[grid_index] = true;
  }
}

void PointExtractor::ResetFeatureOccupancyGrid(
    FeatureOccupancyGrid* feature_occupancy_grid) {
  if (feature_occupancy_grid == nullptr)
    throw std::runtime_error(
        "PointExtractor::InitializeFeatureOccupancyGrid: input pointer is "
        "nullptr");

  auto& occupancy_grid = feature_occupancy_grid->is_occupied_list;
  std::fill(occupancy_grid.begin(), occupancy_grid.end(), false);
}

std::vector<Vec2f> PointExtractor::ExtractWithBucketing(
    const Image& image, const FeatureOccupancyGrid& feature_occupancy_grid) {
  // INPUT IMAGE MUST BE 'CV_8UC1' image.
  if (image.type() != CV_8UC1) {
    throw std::runtime_error(
        "PointExtractor::ExtractWithBinning: input image must be CV_8UC1");
  }

  struct IndexAndScore {
    int index{-1};
    double score{-1.0};
  };

  const double inverse_grid_size = 1.0 / feature_occupancy_grid.grid_size;
  const int num_grid_along_width = feature_occupancy_grid.num_grid_along_width;
  const int num_grid_along_height =
      feature_occupancy_grid.num_grid_along_height;

  // Detect orb feature for the whole image.
  std::vector<cv::KeyPoint> keypoints;
  orb_detector_->detect(image, keypoints);

  std::vector<IndexAndScore> index_score_grid;
  const auto& is_occupied_list = feature_occupancy_grid.is_occupied_list;
  index_score_grid.resize(is_occupied_list.size());

  for (size_t i = 0; i < keypoints.size(); ++i) {
    const auto& keypoint = keypoints[i];

    const int x_index =
        static_cast<int>(std::floor(keypoint.pt.x * inverse_grid_size));
    const int y_index =
        static_cast<int>(std::floor(keypoint.pt.y * inverse_grid_size));
    if (x_index < 0 || x_index >= num_grid_along_width || y_index < 0 ||
        y_index >= num_grid_along_height)
      continue;

    const int grid_index = y_index + x_index * num_grid_along_height;
    if (is_occupied_list[grid_index] == true) continue;

    auto& index_score = index_score_grid[grid_index];
    if (index_score.score <= keypoint.response) {
      index_score.index = i;
      index_score.score = keypoint.response;
    }
  }

  std::vector<Vec2f> extracted_pixels;
  extracted_pixels.reserve(index_score_grid.size());
  for (size_t index = 0; index < is_occupied_list.size(); ++index) {
    const auto& index_score = index_score_grid[index];
    if (index_score.index == -1) continue;

    Vec2f extracted_pixel{Vec2f::Zero()};
    extracted_pixel.x() = keypoints[index_score.index].pt.x;
    extracted_pixel.y() = keypoints[index_score.index].pt.y;
    extracted_pixels.push_back(extracted_pixel);
  }

  std::cout << "# pts (extracted): " << extracted_pixels.size() << std::endl;

  return extracted_pixels;
}

}  // namespace feature_extractor
}  // namespace visual_inertial_odometry