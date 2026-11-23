#pragma once

#include <calibration_workbench/optimizations/covariance_types.h>
#include <calibration_workbench/core/camera_intrinsics.h>
#include <calibration_workbench/core/types.h>

namespace calibration_workbench
{
/**
 * @brief Structure containing the relevant data for a camera intrinsic calibration
 * @ingroup optimizations_camera_intrinsic
 */
struct CameraIntrinsicProblem
{
  std::vector<Correspondence2D3D::Set> image_observations;
  CameraIntrinsics intrinsics_guess;
  bool use_extrinsic_guesses;
  VectorEigenIsometry extrinsic_guesses;

  std::string label_extr = "pose";
  const std::array<std::string, 9> labels_intrinsic_params = { { "fx", "fy", "cx", "cy", "k1", "k2", "p1", "p2",
                                                                 "k3" } };
  const std::array<std::string, 6> labels_isometry3d = { { "x", "y", "z", "rx", "ry", "rz" } };
};

/**
 * @brief Results of the camera intrinsic calibration
 * @details The cost fields store one quarter of the squared per-point pixel reprojection RMS.
 * An unavailable initial cost is stored as positive infinity.
 * @ingroup optimizations_camera_intrinsic
 */
struct CameraIntrinsicResult
{
  bool converged;
  double initial_cost_per_obs;
  double final_cost_per_obs;

  CameraIntrinsics intrinsics;
  std::array<double, 5> distortions;

  VectorEigenIsometry target_transforms;

  CovarianceResult covariance;
};
std::ostream& operator<<(std::ostream& stream, const CameraIntrinsicResult& result);

/**
 * @brief Intrinsic camera parameters from a ROS camera calibration YAML document
 */
struct ROSCameraCalibration
{
  std::string camera_name;
  int image_width;
  int image_height;
  CameraIntrinsics intrinsics;
  std::array<double, 5> distortions;
};

/**
 * @brief Reads the image size, camera name, pinhole intrinsics, and plumb_bob distortion coefficients
 * @throws BadFileException for malformed data or unsupported camera models
 */
ROSCameraCalibration fromROSFormat(const YAML::Node& node);

/**
 * @brief Saves the calibration results to a YAML file in a format compatible with ROS
 * @details Format definition https://wiki.ros.org/camera_calibration_parsers#File_formats
 * @throws Exception on failure
 */
YAML::Node toROSFormat(const CameraIntrinsicResult& result, const int image_width, const int image_height);

/**
 * @brief Performs the camera intrinsic calibration
 * @ingroup optimizations_camera_intrinsic
 */
CameraIntrinsicResult optimize(const CameraIntrinsicProblem& params);

}  // namespace calibration_workbench

namespace YAML
{
class Node;

template <typename T>
struct convert;

template <>
struct convert<calibration_workbench::CameraIntrinsicProblem>
{
  static Node encode(const calibration_workbench::CameraIntrinsicProblem& rhs);
  static bool decode(const Node& node, calibration_workbench::CameraIntrinsicProblem& rhs);
};

template <>
struct convert<calibration_workbench::CameraIntrinsicResult>
{
  static Node encode(const calibration_workbench::CameraIntrinsicResult& rhs);
  static bool decode(const YAML::Node& node, calibration_workbench::CameraIntrinsicResult& rhs);
};

}  // namespace YAML
