#pragma once

#include <calibration_workbench/core/types.h>
#include <calibration_workbench/optimizations/pnp.h>
#include <calibration_workbench/analysis/statistics.h>

namespace calibration_workbench
{
/**
 * @brief This function qualifies 2D sensor noise by
 * comparing PnP results from images taken at same pose.
 * Sensor noise can be understood by inspecting the returned standard
 * deviations.
 * @return Noise Statistics: a vector of means & std devs
 * @throws ICalException when fewer than two PnP solutions converge
 * @ingroup analysis_noise_qualification
 */
PnPNoiseStat qualifyNoise2D(const std::vector<PnPProblem>& params);

/**
 * @brief This function qualifies 3D sensor noise by
 * comparing PnP results from scans taken at the same pose.
 * Sensor noise can be understood by inspecting the returned standard
 * deviations.
 * @return Noise Statiscics: a vector of standard deviations and the mean pos
 * @throws ICalException when fewer than two PnP solutions converge
 * @ingroup analysis_noise_qualification
 */
PnPNoiseStat qualifyNoise3D(const std::vector<PnPProblem3D>& params);

}  // namespace calibration_workbench
