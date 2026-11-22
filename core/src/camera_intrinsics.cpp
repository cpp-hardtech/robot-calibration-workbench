#include <calibration_workbench/core/camera_intrinsics.h>
#include <calibration_workbench/core/serialization.h>

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
std::ostream& operator<<(std::ostream& stream, const CameraIntrinsics& intr)
{
  stream << "Camera intrinsics:\n\tfx = " << intr.fx() << "\n\tfy = " << intr.fy() << "\n\tcx = " << intr.cx()
         << "\n\tcy = " << intr.cy();
  return stream;
}

}  // namespace calibration_workbench
