#pragma once

#include <calibration_workbench/gui/configurable_widget.h>
#include <string>

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

  /** @brief Sets pinhole intrinsics from inline values or a ROS camera calibration file. */
  void configureFromConfig(const YAML::Node& node, const std::string& config_file);

  YAML::Node save() const override;

private:
  Ui::CameraIntrinsics* ui_;
};

}  // namespace calibration_workbench
