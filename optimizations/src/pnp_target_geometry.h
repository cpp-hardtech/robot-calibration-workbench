#pragma once

#include <calibration_workbench/core/exceptions.h>
#include <calibration_workbench/core/types.h>

namespace calibration_workbench::detail
{
class PnPTargetGeometry
{
public:
  void addPoint(const Eigen::Vector3d& point) { points_.push_back(point); }

  void validate() const
  {
    if (points_.size() >= 3)
    {
      Eigen::Matrix3Xd offsets(3, static_cast<Eigen::Index>(points_.size() - 1));
      for (Eigen::Index col = 0; col < offsets.cols(); ++col)
        offsets.col(col) = points_[col + 1] - points_.front();
      if (offsets.fullPivLu().rank() >= 2)
        return;
    }

    throw OptimizationException("PnP requires at least three noncollinear target points");
  }

private:
  VectorVector3<double> points_;
};
}  // namespace calibration_workbench::detail
