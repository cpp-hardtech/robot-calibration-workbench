#include <calibration_workbench/optimizations/camera_intrinsic.h>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <yaml-cpp/yaml.h>

namespace
{
using calibration_workbench::CameraIntrinsicResult;

void checkRendering(const CameraIntrinsicResult& result, bool initial_available)
{
  std::ostringstream rendered;
  rendered << result;
  std::ostringstream expected;
  expected << "Initial reprojection RMS (pixels): ";
  if (initial_available)
    expected << 2.0 * std::sqrt(result.initial_cost_per_obs);
  else
    expected << "unavailable";
  expected << "\nFinal reprojection RMS (pixels): " << 2.0 * std::sqrt(result.final_cost_per_obs);
  if (rendered.str().find(expected.str()) == std::string::npos)
    throw std::runtime_error("Unexpected reprojection RMS output: " + rendered.str());
  std::cout << expected.str() << '\n';
}
}  // namespace

int main(int argc, char** argv)
{
  if (argc != 3)
  {
    std::cerr << "Expected Ceres and OpenCV result files\n";
    return 1;
  }

  try
  {
    const auto ceres = YAML::LoadFile(argv[1]).as<CameraIntrinsicResult>();
    const auto opencv = YAML::LoadFile(argv[2]).as<CameraIntrinsicResult>();
    if (!ceres.converged || !opencv.converged || !std::isfinite(ceres.initial_cost_per_obs) ||
        !std::isfinite(ceres.final_cost_per_obs) || !std::isfinite(opencv.final_cost_per_obs) ||
        ceres.final_cost_per_obs <= 0.0 || opencv.final_cost_per_obs <= 0.0)
      throw std::runtime_error("Expected converged calibrations with finite positive final costs");

    if (std::abs(ceres.final_cost_per_obs - opencv.final_cost_per_obs) > 0.1 * ceres.final_cost_per_obs)
      throw std::runtime_error("Ceres and OpenCV returned incompatible normalized costs for the same observations");
    if (!std::isinf(opencv.initial_cost_per_obs) || opencv.initial_cost_per_obs < 0.0)
      throw std::runtime_error("OpenCV did not preserve the unavailable initial cost");

    std::cout << "Ceres normalized final cost: " << ceres.final_cost_per_obs << '\n';
    checkRendering(ceres, true);
    std::cout << "OpenCV normalized final cost: " << opencv.final_cost_per_obs << '\n';
    checkRendering(opencv, false);
    auto invalid_initial = ceres;
    invalid_initial.initial_cost_per_obs = -1.0;
    checkRendering(invalid_initial, false);
  }
  catch (const std::exception& ex)
  {
    std::cerr << ex.what() << '\n';
    return 1;
  }
  return 0;
}
