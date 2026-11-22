#include "ui_camera_intrinsics.h"
#include <calibration_workbench/gui/camera_intrinsics.h>
#include <calibration_workbench/core/serialization.h>

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
