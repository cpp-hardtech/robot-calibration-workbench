#pragma once

#include <calibration_workbench/gui/configurable_widget.h>

namespace Ui
{
class CharucoGridTargetFinder;
}

namespace calibration_workbench
{
class CharucoGridTargetFinderWidget : public ConfigurableWidget
{
public:
  explicit CharucoGridTargetFinderWidget(QWidget* parent = nullptr);
  ~CharucoGridTargetFinderWidget();

  void configure(const YAML::Node& node) override;
  YAML::Node save() const override;

private:
  Ui::CharucoGridTargetFinder* ui_;
};

}  // namespace calibration_workbench
