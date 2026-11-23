#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>
#include <system_error>

namespace calibration_workbench
{
namespace output_path_identity
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

inline bool aliases(const NamedPath& output, const NamedPath& other, bool check_hard_links)
{
  namespace fs = std::filesystem;
  return output.resolved == other.resolved ||
         (check_hard_links && fs::exists(output.resolved) && fs::exists(other.resolved) &&
          fs::equivalent(output.resolved, other.resolved));
}

inline void requireDistinct(const NamedPath& output, const NamedPath& other)
{
  if (aliases(output, other, true))
    throw std::runtime_error(describe(output) + " conflicts with " + describe(other));
}
}  // namespace output_path_identity
}  // namespace calibration_workbench
