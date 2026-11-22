#include <calibration_workbench/target_finders/opencv/chessboard_grid_target_finder.h>
#include <calibration_workbench/core/exceptions.h>
#include <calibration_workbench/core/serialization.h>

#include <cmath>
#include <limits>
#include <opencv2/calib3d.hpp>

namespace calibration_workbench
{
ChessboardGridTarget::ChessboardGridTarget(int rows, int cols, double spacing)
  : rows(rows), cols(cols), spacing(spacing)
{
  if (rows < 3 || cols < 3)
    throw ICalException("Checkerboard dimensions must define at least three inner corners per side");
  if (cols > std::numeric_limits<int>::max() / rows)
    throw ICalException("Checkerboard corner count exceeds the supported range");
  if (!std::isfinite(spacing) || spacing <= 0.0)
    throw ICalException("Checkerboard corner spacing must be finite and positive");
}

std::size_t ChessboardGridTarget::cornerCount() const
{
  return static_cast<std::size_t>(rows) * static_cast<std::size_t>(cols);
}

Correspondence2D3D::Set ChessboardGridTarget::createCorrespondences(const TargetFeatures2D& target_features) const
{
  Correspondence2D3D::Set correspondences;
  correspondences.reserve(target_features.size());
  for (const auto& [id, observations] : target_features)
  {
    if (id >= cornerCount() || observations.size() != 1)
      throw ICalException("Invalid checkerboard corner observation");
    const Eigen::Vector3d point((id % cols) * spacing, (id / cols) * spacing, 0.0);
    correspondences.emplace_back(observations.front(), point);
  }
  return correspondences;
}

ChessboardGridTargetFinder::ChessboardGridTargetFinder(const ChessboardGridTarget& target) : target_(target) {}

TargetFeatures2D ChessboardGridTargetFinder::findTargetFeatures(const cv::Mat& image) const
{
  std::vector<cv::Point2f> corners;
  if (!cv::findChessboardCornersSB(image, { target_.cols, target_.rows }, corners))
    return {};

  TargetFeatures2D features;
  for (std::size_t i = 0; i < corners.size(); ++i)
    features.emplace(static_cast<unsigned>(i), VectorEigenVector<2>{ Eigen::Vector2d(corners[i].x, corners[i].y) });
  return features;
}

cv::Mat ChessboardGridTargetFinder::drawTargetFeatures(const cv::Mat& image,
                                                       const TargetFeatures2D& target_features) const
{
  cv::Mat overlay = image.clone();
  if (target_features.empty())
    return overlay;

  const auto correspondences = target_.createCorrespondences(target_features);
  std::vector<cv::Point2f> corners;
  corners.reserve(correspondences.size());
  for (const auto& correspondence : correspondences)
    corners.emplace_back(correspondence.in_image.x(), correspondence.in_image.y());

  cv::drawChessboardCorners(overlay, { target_.cols, target_.rows }, corners, corners.size() == target_.cornerCount());
  return overlay;
}

TargetFinderOpenCV::ConstPtr ChessboardGridTargetFinderFactory::create(const YAML::Node& config) const
{
  const auto rows = getMember<int>(config, "rows");
  const auto cols = getMember<int>(config, "cols");
  const auto spacing = getMember<double>(config, "spacing");
  return std::make_shared<const ChessboardGridTargetFinder>(ChessboardGridTarget(rows, cols, spacing));
}
}  // namespace calibration_workbench
