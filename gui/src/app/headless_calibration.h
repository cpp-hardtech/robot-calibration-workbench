#pragma once

#include <filesystem>
#include <iostream>
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

inline void validateObservationReportPath(const char* report_path, const char* const* begin, const char* const* end)
{
  if (!report_path)
    return;

  const auto report = std::filesystem::weakly_canonical(std::filesystem::absolute(report_path));
  for (auto path = begin; path != end; ++path)
  {
    if (report == std::filesystem::weakly_canonical(std::filesystem::absolute(*path)))
      throw std::runtime_error("Observation report path must differ from configuration, observation, and result paths");
  }
}
template <typename Widget>
int runObservationValidation(Widget& widget, char** argv)
{
  try
  {
    validateObservationReportPath(argv[4], argv + 2, argv + 4);
    widget.loadConfig(argv[2]);
    widget.loadObservations(argv[3]);
    const auto usable = widget.validateObservations();
    widget.saveObservationReport(argv[4]);
    if (usable == 0)
    {
      std::cerr << "No usable observations for calibration\n";
      return 1;
    }

    std::cout << "Usable observations: " << usable << '\n';
    return 0;
  }
  catch (const std::exception& ex)
  {
    std::cerr << "Validation error: " << ex.what() << '\n';
    return 1;
  }
}
}  // namespace calibration_workbench
