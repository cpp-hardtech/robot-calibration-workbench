#include <calibration_workbench/core/camera_intrinsics.h>
#include <calibration_workbench/core/exceptions.h>
#include <calibration_workbench/core/serialization.h>

#include <cmath>

using namespace calibration_workbench;

// @cond
namespace YAML
{
Node convert<CameraIntrinsics>::encode(const CameraIntrinsics& rhs)
{
  YAML::Node node;
  node["cx"] = rhs.cx();
  node["cy"] = rhs.cy();
  node["fx"] = rhs.fx();
  node["fy"] = rhs.fy();
  return node;
}

bool convert<CameraIntrinsics>::decode(const YAML::Node& node, CameraIntrinsics& rhs)
{
  rhs.cx() = getMember<double>(node, "cx");
  rhs.cy() = getMember<double>(node, "cy");
  rhs.fx() = getMember<double>(node, "fx");
  rhs.fy() = getMember<double>(node, "fy");

  return true;
}

}  // namespace YAML
// @endcond

namespace calibration_workbench
{
CameraIntrinsics CameraIntrinsics::remapImageCoordinates(double scale_x, double scale_y, double offset_x,
                                                         double offset_y) const
{
  if (!std::isfinite(scale_x) || !std::isfinite(scale_y) || !std::isfinite(offset_x) || !std::isfinite(offset_y) ||
      scale_x <= 0.0 || scale_y <= 0.0)
    throw ICalException("Image coordinate remapping requires finite offsets and positive, finite scales");

  const CameraIntrinsics result{ { scale_x * fx(), scale_y * fy(), scale_x * cx() + offset_x,
                                   scale_y * cy() + offset_y } };
  for (double value : result.values)
  {
    if (!std::isfinite(value))
      throw ICalException("Remapped camera intrinsics must be finite");
  }

  return result;
}

std::ostream& operator<<(std::ostream& stream, const CameraIntrinsics& intr)
{
  stream << "Camera intrinsics:\n\tfx = " << intr.fx() << "\n\tfy = " << intr.fy() << "\n\tcx = " << intr.cx()
         << "\n\tcy = " << intr.cy();
  return stream;
}

}  // namespace calibration_workbench
