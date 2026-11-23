#include <QApplication>
#include <calibration_workbench/gui/target_finder.h>
#include <calibration_workbench/target_finders/opencv/circle_detector.h>
#include <array>
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace
{
using calibration_workbench::CircleDetectorParams;

bool hasDetectorDefaults(const YAML::Node& config)
{
  const auto actual = config["circle_detector_params"].as<CircleDetectorParams>();
  const CircleDetectorParams defaults;
  return actual.minThreshold == defaults.minThreshold && actual.maxThreshold == defaults.maxThreshold &&
         actual.nThresholds == defaults.nThresholds && actual.minRepeatability == defaults.minRepeatability &&
         actual.circleInclusionRadius == defaults.circleInclusionRadius &&
         actual.maxRadiusDiff == defaults.maxRadiusDiff &&
         actual.maxAverageEllipseError == defaults.maxAverageEllipseError &&
         actual.filterByColor == defaults.filterByColor && actual.circleColor == defaults.circleColor &&
         actual.filterByArea == defaults.filterByArea && actual.minArea == defaults.minArea &&
         actual.maxArea == defaults.maxArea && actual.filterByCircularity == defaults.filterByCircularity &&
         actual.minCircularity == defaults.minCircularity && actual.maxCircularity > 1.0f &&
         actual.filterByInertia == defaults.filterByInertia && actual.minInertiaRatio == defaults.minInertiaRatio &&
         actual.maxInertiaRatio > 1.0f && actual.filterByConvexity == defaults.filterByConvexity &&
         actual.minConvexity == defaults.minConvexity && actual.maxConvexity > 1.0f;
}
}  // namespace

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cerr << "Expected a sample configuration file\n";
    return 1;
  }

  QApplication app(argc, argv);
  calibration_workbench::TargetFinderWidget finder;
  auto target = YAML::LoadFile(argv[1])["target_finder"];
  auto detector = target["circle_detector_params"];
  const std::array<bool, 2> states{ false, true };
  bool preserved = true;
  for (const bool circularity : states)
  {
    for (const bool convexity : states)
    {
      detector["filterByCircularity"] = circularity;
      detector["filterByConvexity"] = convexity;
      finder.configure(target);

      const auto saved = finder.save()["circle_detector_params"];
      const bool saved_circularity = saved["filterByCircularity"].as<bool>();
      const bool saved_convexity = saved["filterByConvexity"].as<bool>();
      std::cout << std::boolalpha << "Input circularity=" << circularity << " convexity=" << convexity
                << ", saved circularity=" << saved_circularity << " convexity=" << saved_convexity << '\n';
      preserved = preserved && saved_circularity == circularity && saved_convexity == convexity;
    }
  }
  detector["minThreshold"] = 20.25;
  detector["maxThreshold"] = 220.75;
  detector["nThresholds"] = 9;
  detector["minRepeatability"] = 2;
  detector["filterByColor"] = false;
  detector["filterByArea"] = false;
  detector["filterByInertia"] = true;
  detector["filterByConvexity"] = false;
  finder.configure(target);
  const auto customized = finder.save();
  const auto fractional = customized["circle_detector_params"].as<CircleDetectorParams>();
  const bool thresholds_preserved = fractional.minThreshold == 20.25f && fractional.maxThreshold == 220.75f;
  std::cout << "Fractional thresholds saved: " << fractional.minThreshold << "/" << fractional.maxThreshold << '\n';

  target.remove("circle_detector_params");
  finder.configure(target);
  const auto default_config = finder.save();
  const bool restored_defaults = hasDetectorDefaults(default_config);
  finder.configure(customized);
  finder.configure(default_config);
  const bool reloaded_defaults = hasDetectorDefaults(finder.save());
  std::cout << "Omitted detector parameters restored defaults: " << restored_defaults
            << ", saved defaults reloaded: " << reloaded_defaults << '\n';
  return preserved && thresholds_preserved && restored_defaults && reloaded_defaults ? 0 : 1;
}
