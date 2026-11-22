#pragma once

#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string_view>

namespace calibration_workbench
{
struct HeadlessOutputArguments
{
  int positional_argc;
  const char* observation_report;
};

inline std::optional<HeadlessOutputArguments> parseHeadlessOutputArguments(int argc, char** argv, int max_argc)
{
  HeadlessOutputArguments output{ argc, nullptr };
  for (int i = 2; i < argc; ++i)
  {
    if (std::string_view(argv[i]) != "--observation-report")
      continue;

    if (i != argc - 2 || argv[i + 1][0] == '\0' || std::string_view(argv[i + 1]) == "--observation-report")
      return std::nullopt;

    output = { i, argv[i + 1] };
    break;
  }

  if (output.positional_argc < 5 || output.positional_argc > max_argc)
    return std::nullopt;
  return output;
}

inline void validateObservationReportPath(const HeadlessOutputArguments& output, char** argv)
{
  if (!output.observation_report)
    return;

  const auto report = std::filesystem::weakly_canonical(std::filesystem::absolute(output.observation_report));
  for (int i = 2; i < output.positional_argc; ++i)
  {
    if (report == std::filesystem::weakly_canonical(std::filesystem::absolute(argv[i])))
      throw std::runtime_error("Observation report path must differ from configuration, observation, and result paths");
  }
}
}  // namespace calibration_workbench
