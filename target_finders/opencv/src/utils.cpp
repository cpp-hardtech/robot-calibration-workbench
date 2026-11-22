#include <calibration_workbench/target_finders/opencv/utils.h>
#include <calibration_workbench/core/exceptions.h>
#include <calibration_workbench/core/serialization.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

namespace calibration_workbench
{
void drawReprojections(const VectorVector2d& reprojections, int size, cv::Scalar color, cv::Mat& image)
{
  for (const auto& pt : reprojections)
  {
    cv::circle(image, cv::Point2d(pt.x(), pt.y()), size, color);
  }
}

cv::Mat readImageOpenCV(const std::string& path)
{
  cv::Mat image = cv::imread(path, cv::IMREAD_COLOR);
  if (image.data == nullptr)
    throw BadFileException("Failed to load file at: '" + path + "'");

  return image;
}

}  // namespace calibration_workbench
