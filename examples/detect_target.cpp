#include <calibration_workbench/core/exceptions.h>
#include <calibration_workbench/core/serialization.h>
#include <calibration_workbench/target_finders/opencv/target_finder.h>
#include <calibration_workbench/target_finders/opencv/utils.h>

#include <boost_plugin_loader/plugin_loader.hpp>

#include <exception>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <stdexcept>
#include <string>
#include <string_view>

#include <opencv2/imgcodecs.hpp>

namespace
{
using calibration_workbench::TargetFinderOpenCV;

struct ImageReport
{
  std::string_view status;
  std::size_t feature_count = 0;
  std::size_t point_count = 0;
};

ImageReport inspectImage(const TargetFinderOpenCV::ConstPtr& finder, const std::string& image_path)
{
  cv::Mat image;
  try
  {
    image = calibration_workbench::readImageOpenCV(image_path);
  }
  catch (const std::exception& error)
  {
    std::cerr << "Cannot read " << image_path << ": " << error.what() << '\n';
    return { "unreadable" };
  }

  try
  {
    const auto features = finder->findTargetFeatures(image);
    if (features.empty())
      return { "no_detection" };

    std::size_t point_count = 0;
    for (const auto& feature : features)
      point_count += feature.second.size();

    return { "detected", features.size(), point_count };
  }
  catch (const calibration_workbench::TargetNotFoundException& error)
  {
    std::cerr << image_path << ": " << error.what() << '\n';
    return { "no_detection" };
  }
  catch (const std::exception& error)
  {
    std::cerr << image_path << ": " << error.what() << '\n';
    return { "detector_error" };
  }
}

int runBatch(const TargetFinderOpenCV::ConstPtr& finder, int argc, char** argv)
{
  bool all_detected = true;
  std::cout.imbue(std::locale::classic());
  std::cout << "image_path,status,feature_count,point_count\n";
  for (int i = 3; i < argc; ++i)
  {
    const std::string image_path(argv[i]);
    const auto report = inspectImage(finder, image_path);
    all_detected &= report.status == "detected";

    std::cout << '"';
    for (const char character : image_path)
    {
      if (character == '"')
        std::cout << '"';
      std::cout << character;
    }
    std::cout << "\"," << report.status << ',' << report.feature_count << ',' << report.point_count << '\n';
  }

  if (!std::cout.flush())
  {
    std::cerr << "Failed to write target CSV\n";
    return 1;
  }
  return all_detected ? 0 : 1;
}
}  // namespace

int main(int argc, char** argv)
{
  constexpr auto usage = "Usage: calibration_workbench_detect_target <target.yaml> <image> [--overlay <path>] "
                         "[--correspondences]\n"
                         "       calibration_workbench_detect_target <target.yaml> --batch <image> [<image> ...]\n";
  if (argc == 2 && std::string_view(argv[1]) == "--help")
  {
    std::cout << usage;
    return 0;
  }
  if (argc < 3)
  {
    std::cerr << usage;
    return 2;
  }

  const bool batch = std::string_view(argv[2]) == "--batch";
  if (batch && argc == 3)
  {
    std::cerr << usage;
    return 2;
  }

  std::string overlay_path;
  bool write_correspondences = false;
  for (int i = 3; i < argc; ++i)
  {
    const std::string_view option(argv[i]);
    if (batch)
    {
      if (option.substr(0, 2) == "--")
      {
        std::cerr << "Batch mode does not accept option " << option << '\n' << usage;
        return 2;
      }
      continue;
    }

    if (option == "--correspondences" && !write_correspondences)
    {
      write_correspondences = true;
      continue;
    }
    if (option == "--overlay" && overlay_path.empty() && i + 1 < argc)
    {
      const std::string_view path(argv[i + 1]);
      if (!path.empty() && path.substr(0, 2) != "--")
      {
        overlay_path = path;
        ++i;
        continue;
      }
    }
    std::cerr << "Invalid option or missing value: " << option << '\n' << usage;
    return 2;
  }

  try
  {
    using namespace calibration_workbench;
    const YAML::Node input = YAML::LoadFile(argv[1]);
    const YAML::Node config = input["target_finder"] ? input["target_finder"] : input;
    const auto plugin_type = getMember<std::string>(config, "type");

    boost_plugin_loader::PluginLoader loader;
    loader.search_libraries.insert(CALIBRATION_WORKBENCH_PLUGIN_LIBRARIES);
    loader.search_libraries_env = CALIBRATION_WORKBENCH_SEARCH_LIBRARIES_ENV;
    const auto factory = loader.createInstance<TargetFinderFactoryOpenCV>(plugin_type);
    const auto finder = factory->create(config);
    if (batch)
      return runBatch(finder, argc, argv);

    const cv::Mat image = readImageOpenCV(argv[2]);
    const auto features = finder->findTargetFeatures(image);
    if (features.empty())
    {
      std::cerr << "No target features detected in " << argv[2] << '\n';
      return 1;
    }

    Correspondence2D3D::Set correspondences;
    if (write_correspondences)
    {
      correspondences = finder->target().createCorrespondences(features);
      if (correspondences.empty())
        throw std::runtime_error("No target correspondences available");
    }

    if (!overlay_path.empty())
    {
      std::error_code comparison_error;
      if (std::filesystem::equivalent(overlay_path, argv[1], comparison_error) ||
          std::filesystem::equivalent(overlay_path, argv[2], comparison_error))
        throw std::runtime_error("Overlay path must differ from the input configuration and image");

      const cv::Mat overlay = finder->drawTargetFeatures(image, features);
      if (overlay.empty() || !cv::imwrite(overlay_path, overlay))
        throw std::runtime_error("Failed to write target overlay: " + overlay_path);
    }

    std::cout.imbue(std::locale::classic());
    std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);
    if (write_correspondences)
    {
      std::cout << "image_x,image_y,target_x,target_y,target_z\n";
      for (const auto& correspondence : correspondences)
        std::cout << correspondence.in_image.x() << ',' << correspondence.in_image.y() << ','
                  << correspondence.in_target.x() << ',' << correspondence.in_target.y() << ','
                  << correspondence.in_target.z() << '\n';
    }
    else
    {
      std::cout << "feature_id,point_index,x,y\n";
      for (const auto& [feature_id, points] : features)
      {
        for (std::size_t i = 0; i < points.size(); ++i)
          std::cout << feature_id << ',' << i << ',' << points[i].x() << ',' << points[i].y() << '\n';
      }
    }
    if (!std::cout.flush())
    {
      std::cerr << "Failed to write target CSV\n";
      return 1;
    }
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
