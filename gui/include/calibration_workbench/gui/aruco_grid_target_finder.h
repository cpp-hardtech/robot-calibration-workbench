#pragma once

#include <calibration_workbench/gui/configurable_widget.h>

namespace Ui
{
class ArucoGridTargetFinder;
}

namespace calibration_workbench
{
class ArucoGridTargetFinderWidget : public ConfigurableWidget
{
public:
  explicit ArucoGridTargetFinderWidget(QWidget* parent = nullptr);
  ~ArucoGridTargetFinderWidget();

  void configure(const YAML::Node& node) override;
  YAML::Node save() const override;

private:
  Ui::ArucoGridTargetFinder* ui_;
};

}  // namespace calibration_workbench
