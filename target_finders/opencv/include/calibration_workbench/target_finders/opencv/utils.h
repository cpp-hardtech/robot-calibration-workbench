#pragma once

#include <calibration_workbench/core/camera_intrinsics.h>
#include <calibration_workbench/core/types.h>

#include <opencv2/core/mat.hpp>

namespace calibration_workbench
{
using VectorVector2d = std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;

/**
 * @brief Draws reprojected points onto an image
 */
void drawReprojections(const VectorVector2d& reprojections, int size, cv::Scalar color, cv::Mat& image);

/**
 * @brief Reads and image from file into an OpenCV matrix
 */
cv::Mat readImageOpenCV(const std::string& path);

}  // namespace calibration_workbench
