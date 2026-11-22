#pragma once

#include <calibration_workbench/core/exceptions.h>

#include <opencv2/aruco.hpp>

#include <cstdint>
#include <string>

namespace calibration_workbench::detail
{
enum class MarkerGridLayout
{
  Full,
  Alternating
};

inline cv::Ptr<cv::aruco::Dictionary> markerBoardDictionary(int dictionary_id, int rows, int cols,
                                                            MarkerGridLayout layout)
{
  if (dictionary_id < cv::aruco::DICT_4X4_50 || dictionary_id > cv::aruco::DICT_APRILTAG_36h11)
    throw ICalException("Unknown OpenCV predefined marker dictionary ID " + std::to_string(dictionary_id));
  if (rows <= 0 || cols <= 0)
    throw ICalException("Marker board dimensions must be positive");

  const auto cells = static_cast<std::int64_t>(rows) * cols;
  const auto required = layout == MarkerGridLayout::Alternating ? cells / 2 : cells;
  const auto dictionary =
      cv::aruco::getPredefinedDictionary(static_cast<cv::aruco::PREDEFINED_DICTIONARY_NAME>(dictionary_id));
  const int available = dictionary->bytesList.rows;
  if (required > available)
    throw ICalException("Marker board requires " + std::to_string(required) + " markers but dictionary " +
                        std::to_string(dictionary_id) + " provides " + std::to_string(available));

  return dictionary;
}
}  // namespace calibration_workbench::detail
