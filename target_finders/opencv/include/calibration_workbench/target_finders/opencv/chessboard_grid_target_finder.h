#pragma once

#include <calibration_workbench/target_finders/opencv/target_finder.h>

namespace calibration_workbench
{
struct ChessboardGridTarget : Target2D3D
{
  ChessboardGridTarget(int rows, int cols, double spacing);

  std::size_t cornerCount() const;
  Correspondence2D3D::Set createCorrespondences(const TargetFeatures2D& target_features) const override;

  const int rows;
  const int cols;
  const double spacing;
};

class ChessboardGridTargetFinder : public TargetFinderOpenCV
{
public:
  explicit ChessboardGridTargetFinder(const ChessboardGridTarget& target);

  TargetFeatures2D findTargetFeatures(const cv::Mat& image) const override;
  cv::Mat drawTargetFeatures(const cv::Mat& image, const TargetFeatures2D& target_features) const override;
  const Target2D3D& target() const override { return target_; }

private:
  const ChessboardGridTarget target_;
};

struct ChessboardGridTargetFinderFactory : TargetFinderFactoryOpenCV
{
  TargetFinderOpenCV::ConstPtr create(const YAML::Node& config) const override;
};
}  // namespace calibration_workbench
