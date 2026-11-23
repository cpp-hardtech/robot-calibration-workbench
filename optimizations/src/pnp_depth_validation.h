#pragma once

#include <calibration_workbench/core/exceptions.h>
#include <calibration_workbench/core/types.h>

#include <string>

namespace calibration_workbench::detail
{
inline void validatePnPDepth(const Eigen::Isometry3d& target_to_camera, const Correspondence2D3D::Set& observations,
                             std::size_t camera_index)
{
  for (std::size_t i = 0; i < observations.size(); ++i)
  {
    const Eigen::Vector3d camera_point = target_to_camera * observations[i].in_target;
    if (!camera_point.allFinite() || camera_point.z() <= 0.0)
      throw OptimizationException("PnP solution requires a finite camera point with positive depth at camera " +
                                  std::to_string(camera_index) + ", observation " + std::to_string(i));
  }
}
}  // namespace calibration_workbench::detail
