#pragma once

#include "../output_path_identity.h"

namespace calibration_workbench
{
inline void validateHeadlessResultPaths(const char* config, const char* observations, const char* result,
                                        const char* ros = nullptr)
{
  using output_path_identity::requireDistinct;
  using output_path_identity::resolvePath;
  const auto config_path = resolvePath("configuration", config);
  const auto observations_path = resolvePath("observation manifest", observations);
  const auto result_path = resolvePath("calibration result", result);
  requireDistinct(result_path, config_path);
  requireDistinct(result_path, observations_path);

  if (ros)
  {
    const auto ros_path = resolvePath("ROS calibration result", ros);
    requireDistinct(ros_path, config_path);
    requireDistinct(ros_path, observations_path);
    requireDistinct(ros_path, result_path);
  }
}
}  // namespace calibration_workbench
