#include <calibration_workbench/core/serialization.h>
#include <calibration_workbench/optimizations/camera_intrinsic.h>
#include <calibration_workbench/target_finders/opencv/target_finder.h>
#include <calibration_workbench/target_finders/opencv/utils.h>

#include <boost_plugin_loader/plugin_loader.hpp>
#include <opencv2/calib3d.hpp>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace
{
using namespace calibration_workbench;

YAML::Node estimatePose(const ROSCameraCalibration& camera, const Correspondence2D3D::Set& correspondences)
{
  if (correspondences.size() < 4)
    throw std::runtime_error("At least four target correspondences are required");

  std::vector<cv::Point3d> target_points;
  std::vector<cv::Point2d> image_points;
  target_points.reserve(correspondences.size());
  image_points.reserve(correspondences.size());
  for (const auto& correspondence : correspondences)
  {
    if (!correspondence.in_target.allFinite() || !correspondence.in_image.allFinite())
      throw std::runtime_error("Target correspondences must be finite");
    target_points.emplace_back(correspondence.in_target.x(), correspondence.in_target.y(),
                               correspondence.in_target.z());
    image_points.emplace_back(correspondence.in_image.x(), correspondence.in_image.y());
  }

  const auto& intrinsics = camera.intrinsics;
  const cv::Matx33d camera_matrix(intrinsics.fx(), 0.0, intrinsics.cx(), 0.0, intrinsics.fy(), intrinsics.cy(), 0.0,
                                  0.0, 1.0);
  const cv::Vec<double, 5> distortion(camera.distortions.data());
  cv::Vec3d rotation_vector, translation;
  if (!cv::solvePnP(target_points, image_points, camera_matrix, distortion, rotation_vector, translation))
    throw std::runtime_error("Unable to estimate a target pose");

  cv::Matx33d rotation;
  cv::Rodrigues(rotation_vector, rotation);
  Eigen::Isometry3d target_to_camera = Eigen::Isometry3d::Identity();
  target_to_camera.linear() = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(rotation.val);
  target_to_camera.translation() = Eigen::Vector3d(translation[0], translation[1], translation[2]);
  if (!target_to_camera.matrix().allFinite())
    throw std::runtime_error("The estimated target pose is not finite");
  for (const auto& correspondence : correspondences)
  {
    const Eigen::Vector3d camera_point = target_to_camera * correspondence.in_target;
    if (!camera_point.allFinite() || camera_point.z() <= 0.0)
      throw std::runtime_error("The estimated target pose places a point outside positive camera depth");
  }

  std::vector<cv::Point2d> projected_points;
  cv::projectPoints(target_points, rotation_vector, translation, camera_matrix, distortion, projected_points);
  double squared_error = 0.0;
  for (std::size_t i = 0; i < image_points.size(); ++i)
  {
    const cv::Point2d residual = projected_points[i] - image_points[i];
    squared_error += residual.dot(residual);
  }
  const double rms = std::sqrt(squared_error / static_cast<double>(image_points.size()));
  if (!std::isfinite(rms))
    throw std::runtime_error("The estimated target pose has a non-finite reprojection error");

  YAML::Node report;
  report["target_to_camera"] = target_to_camera;
  report["correspondence_count"] = correspondences.size();
  report["reprojection_rms_px"] = rms;
  return report;
}
}  // namespace

int main(int argc, char** argv)
{
  constexpr auto usage = "Usage: calibration_workbench_estimate_target_pose <target.yaml> <camera.yaml> <image>\n";
  if (argc == 2 && std::string_view(argv[1]) == "--help")
  {
    std::cout << usage;
    return 0;
  }
  if (argc != 4)
  {
    std::cerr << usage;
    return 2;
  }

  try
  {
    using namespace calibration_workbench;
    const YAML::Node input = YAML::LoadFile(argv[1]);
    const YAML::Node target = input["target_finder"] ? input["target_finder"] : input;
    const auto camera = fromROSFormat(YAML::LoadFile(argv[2]));
    const cv::Mat image = readImageOpenCV(argv[3]);
    if (image.cols != camera.image_width || image.rows != camera.image_height)
      throw std::runtime_error("Image dimensions must match the ROS camera calibration");

    boost_plugin_loader::PluginLoader loader;
    loader.search_libraries.insert(CALIBRATION_WORKBENCH_PLUGIN_LIBRARIES);
    loader.search_libraries_env = CALIBRATION_WORKBENCH_SEARCH_LIBRARIES_ENV;
    const auto factory = loader.createInstance<TargetFinderFactoryOpenCV>(getMember<std::string>(target, "type"));
    const auto finder = factory->create(target);
    const YAML::Node report = estimatePose(camera, finder->findCorrespondences(image));

    YAML::Emitter emitter;
    emitter.SetDoublePrecision(std::numeric_limits<double>::max_digits10);
    emitter << report;
    if (!emitter.good())
      throw std::runtime_error("Unable to serialize the target pose");
    std::cout << emitter.c_str() << '\n';
    if (!std::cout.flush())
      throw std::runtime_error("Failed to write the target pose");
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
