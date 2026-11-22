#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>
#include <system_error>

namespace calibration_workbench
{
namespace headless_result_paths
{
struct NamedPath
{
  const char* role;
  const char* argument;
  std::filesystem::path resolved;
};

inline std::string describe(const NamedPath& path) { return std::string(path.role) + " path '" + path.argument + "'"; }

inline NamedPath resolvePath(const char* role, const char* argument)
{
  namespace fs = std::filesystem;
  NamedPath path{ role, argument, {} };
  if (argument[0] == '\0')
    throw std::runtime_error(describe(path) + " is empty");

  try
  {
    fs::path candidate = fs::absolute(argument);
    for (unsigned links = 0;; ++links)
    {
      std::error_code ec;
      if (!fs::is_symlink(fs::symlink_status(candidate, ec)))
      {
        path.resolved = fs::weakly_canonical(candidate);
        return path;
      }
      if (links == 40)
        break;
      candidate = candidate.parent_path() / fs::read_symlink(candidate);
    }
  }
  catch (const fs::filesystem_error& ex)
  {
    throw std::runtime_error("Cannot resolve " + describe(path) + ": " + ex.what());
  }

  throw std::runtime_error("Too many symbolic links in " + describe(path));
}

inline void requireDistinct(const NamedPath& output, const NamedPath& other)
{
  namespace fs = std::filesystem;
  if (output.resolved == other.resolved ||
      (fs::exists(output.resolved) && fs::exists(other.resolved) && fs::equivalent(output.resolved, other.resolved)))
    throw std::runtime_error(describe(output) + " conflicts with " + describe(other));
}
}  // namespace headless_result_paths

inline void validateHeadlessResultPaths(const char* config, const char* observations, const char* result,
                                        const char* ros = nullptr)
{
  using headless_result_paths::requireDistinct;
  using headless_result_paths::resolvePath;
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
