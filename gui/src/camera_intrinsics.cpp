#include "ui_camera_intrinsics.h"
#include <calibration_workbench/gui/camera_intrinsics.h>
#include <calibration_workbench/core/serialization.h>
#include <calibration_workbench/optimizations/camera_intrinsic.h>
#include <QDir>
#include <QFileInfo>

namespace calibration_workbench
{
CameraIntrinsicsWidget::CameraIntrinsicsWidget(QWidget* parent)
  : ConfigurableWidget(parent), ui_(new Ui::CameraIntrinsics)
{
  ui_->setupUi(this);
}

CameraIntrinsicsWidget::~CameraIntrinsicsWidget() { delete ui_; }

void CameraIntrinsicsWidget::configure(const YAML::Node& node)
{
  ui_->double_spin_box_fx->setValue(getMember<double>(node, "fx"));
  ui_->double_spin_box_fy->setValue(getMember<double>(node, "fy"));
  ui_->double_spin_box_cx->setValue(getMember<double>(node, "cx"));
  ui_->double_spin_box_cy->setValue(getMember<double>(node, "cy"));
}

void CameraIntrinsicsWidget::configureFromConfig(const YAML::Node& node, const std::string& config_file)
{
  if (!node.IsMap() || !node["ros_camera_info"].IsDefined())
  {
    configure(node);
    return;
  }

  if (node.size() != 1)
    throw ICalException("ros_camera_info must be the only entry in the intrinsics mapping");

  const auto reference = node["ros_camera_info"];
  if (!reference.IsScalar() || reference.Scalar().empty())
    throw ICalException("ros_camera_info must be a non-empty file path");

  const auto path = QFileInfo(QString::fromStdString(config_file))
                        .absoluteDir()
                        .filePath(QString::fromStdString(reference.Scalar()))
                        .toStdString();
  try
  {
    const auto calibration = fromROSFormat(YAML::LoadFile(path));
    configure(YAML::Node(calibration.intrinsics));
  }
  catch (const std::exception& ex)
  {
    throw ICalException("Failed to load ROS camera calibration '" + path + "': " + ex.what());
  }
}

YAML::Node CameraIntrinsicsWidget::save() const
{
  YAML::Node node;
  node["fx"] = ui_->double_spin_box_fx->value();
  node["fy"] = ui_->double_spin_box_fy->value();
  node["cx"] = ui_->double_spin_box_cx->value();
  node["cy"] = ui_->double_spin_box_cy->value();

  return node;
}

}  // namespace calibration_workbench
