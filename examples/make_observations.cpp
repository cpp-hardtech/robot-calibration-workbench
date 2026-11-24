#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <initializer_list>
#include <iostream>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

namespace
{
namespace fs = std::filesystem;
using FileIndex = std::map<std::string, fs::path>;

fs::path resolveDirectory(const fs::path& directory, const std::string& kind)
{
  if (!fs::is_directory(directory))
    throw std::runtime_error("Expected an existing " + kind + " directory: " + directory.string());
  return fs::canonical(directory);
}

FileIndex scanFiles(const fs::path& directory, const std::string& kind, bool recursive,
                    std::initializer_list<std::string_view> suffixes)
{
  const fs::path root = resolveDirectory(directory, kind);
  FileIndex files;
  const auto add_file = [&](const fs::directory_entry& entry) {
    if (!entry.is_regular_file())
      return;

    const fs::path& path = entry.path();
    std::string suffix = path.extension().string();
    std::transform(suffix.begin(), suffix.end(), suffix.begin(),
                   [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
    if (std::find(suffixes.begin(), suffixes.end(), suffix) == suffixes.end())
      return;

    fs::path relative = path.lexically_relative(root);
    relative.replace_extension();
    const std::string stem = relative.generic_string();
    const auto [existing, inserted] = files.emplace(stem, path);
    if (!inserted)
      throw std::runtime_error("Duplicate " + kind + " stem '" + stem + "': " + existing->second.string() + " and " +
                               path.string());
  };

  if (recursive)
  {
    for (const auto& entry : fs::recursive_directory_iterator(root))
      add_file(entry);
  }
  else
  {
    for (const auto& entry : fs::directory_iterator(root))
      add_file(entry);
  }
  if (files.empty())
    throw std::runtime_error("No " + kind + " files found in " + directory.string());
  return files;
}

std::string manifestPath(const fs::path& path, const std::optional<fs::path>& relative_to)
{
  if (!relative_to)
    return path.generic_string();

  const fs::path relative = path.lexically_relative(*relative_to);
  if (relative.empty())
    throw std::runtime_error("Cannot make " + path.string() + " relative to " + relative_to->string());
  return relative.generic_string();
}
}  // namespace

int main(int argc, char** argv)
{
  constexpr auto usage = "Usage: calibration_workbench_make_observations <image-dir> [--poses <pose-dir>] "
                         "[--relative-to <manifest-directory>] [--recursive]\n";
  if (argc == 2 && std::string_view(argv[1]) == "--help")
  {
    std::cout << usage;
    return 0;
  }
  if (argc < 2 || std::string_view(argv[1]).empty() || std::string_view(argv[1]).substr(0, 2) == "--")
  {
    std::cerr << usage;
    return 2;
  }

  std::optional<fs::path> pose_directory, relative_to;
  bool recursive = false;
  for (int i = 2; i < argc; ++i)
  {
    const std::string_view option(argv[i]);
    if (option == "--recursive" && !recursive)
    {
      recursive = true;
      continue;
    }
    const std::string_view value(i + 1 < argc ? argv[i + 1] : "");
    if (!value.empty() && value.substr(0, 2) != "--")
    {
      if (option == "--poses" && !pose_directory)
      {
        pose_directory = argv[++i];
        continue;
      }
      if (option == "--relative-to" && !relative_to)
      {
        relative_to = argv[++i];
        continue;
      }
    }
    std::cerr << "Invalid option or missing directory: " << option << '\n' << usage;
    return 2;
  }

  try
  {
    const auto images =
        scanFiles(argv[1], "image", recursive,
                  { ".bmp", ".jpeg", ".jpg", ".pbm", ".pgm", ".png", ".pnm", ".ppm", ".tif", ".tiff", ".webp" });
    const auto poses =
        pose_directory ? scanFiles(*pose_directory, "pose", recursive, { ".yaml", ".yml" }) : FileIndex{};
    if (relative_to)
      relative_to = resolveDirectory(*relative_to, "manifest");

    YAML::Node data(YAML::NodeType::Sequence);
    for (const auto& [stem, image] : images)
    {
      YAML::Node observation;
      observation["image"] = manifestPath(image, relative_to);
      if (pose_directory)
      {
        const auto match = poses.find(stem);
        if (match == poses.end())
          throw std::runtime_error("Missing pose for image " + image.string());
        observation["pose"] = manifestPath(match->second, relative_to);
      }
      data.push_back(observation);
    }

    YAML::Node manifest;
    manifest["data"] = data;
    YAML::Emitter emitter;
    emitter << manifest;
    if (!emitter.good())
      throw std::runtime_error("Unable to serialize the observations");
    std::cout << emitter.c_str() << '\n';
    if (!std::cout.flush())
      throw std::runtime_error("Failed to write the observations");
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
  return 0;
}
