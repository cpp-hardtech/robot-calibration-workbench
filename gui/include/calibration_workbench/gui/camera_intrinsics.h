#pragma once

#include <calibration_workbench/gui/configurable_widget.h>

namespace Ui
{
class CameraIntrinsics;
}

namespace calibration_workbench
{
class CameraIntrinsicsWidget : public ConfigurableWidget
{
public:
  explicit CameraIntrinsicsWidget(QWidget* parent = nullptr);
  ~CameraIntrinsicsWidget();

  void configure(const YAML::Node& node) override;
  YAML::Node save() const override;

private:
  Ui::CameraIntrinsics* ui_;
};

}  // namespace calibration_workbench
