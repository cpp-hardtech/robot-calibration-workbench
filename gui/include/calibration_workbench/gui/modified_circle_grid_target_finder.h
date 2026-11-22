#pragma once

#include <calibration_workbench/gui/configurable_widget.h>

namespace Ui
{
class ModifiedCircleGridTargetFinder;
}

namespace calibration_workbench
{
class ModifiedCircleGridTargetFinderWidget : public ConfigurableWidget
{
public:
  explicit ModifiedCircleGridTargetFinderWidget(QWidget* parent = nullptr);
  ~ModifiedCircleGridTargetFinderWidget();

  void configure(const YAML::Node& node) override;
  YAML::Node save() const override;

private:
  Ui::ModifiedCircleGridTargetFinder* ui_;
};

}  // namespace calibration_workbench
