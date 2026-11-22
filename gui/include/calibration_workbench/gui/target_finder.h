#pragma once

#include <calibration_workbench/gui/configurable_widget.h>

#include <yaml-cpp/yaml.h>

namespace Ui
{
class TargetFinder;
}

namespace calibration_workbench
{
class TargetFinderWidget : public ConfigurableWidget
{
public:
  TargetFinderWidget(QWidget* parent = nullptr);
  virtual ~TargetFinderWidget();

  void configure(const YAML::Node& node) override;
  YAML::Node save() const override;

private:
  Ui::TargetFinder* ui_;
};

}  // namespace calibration_workbench
