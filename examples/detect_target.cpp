#include <calibration_workbench/core/serialization.h>
#include <calibration_workbench/target_finders/opencv/target_finder.h>
#include <calibration_workbench/target_finders/opencv/utils.h>

#include <boost_plugin_loader/plugin_loader.hpp>

#include <exception>
#include <iomanip>
#include <iostream>
#include <limits>
#include <locale>
#include <string>
#include <string_view>

int main(int argc, char** argv)
{
  constexpr auto usage = "Usage: calibration_workbench_detect_target <target.yaml> <image>\n";
  if (argc == 2 && std::string_view(argv[1]) == "--help")
  {
    std::cout << usage;
    return 0;
  }
  if (argc != 3)
  {
    std::cerr << usage;
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
    const auto features = finder->findTargetFeatures(readImageOpenCV(argv[2]));
    if (features.empty())
    {
      std::cerr << "No target features detected in " << argv[2] << '\n';
      return 1;
    }

    std::cout.imbue(std::locale::classic());
    std::cout << std::setprecision(std::numeric_limits<double>::max_digits10);
    std::cout << "feature_id,point_index,x,y\n";
    for (const auto& [feature_id, points] : features)
    {
      for (std::size_t i = 0; i < points.size(); ++i)
        std::cout << feature_id << ',' << i << ',' << points[i].x() << ',' << points[i].y() << '\n';
    }
    if (!std::cout.flush())
    {
      std::cerr << "Failed to write target detections\n";
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
