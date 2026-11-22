#pragma once

#include <calibration_workbench/gui/configurable_widget.h>

namespace Ui
{
class TransformGuess;
}

namespace calibration_workbench
{
class TransformGuess : public ConfigurableWidget
{
public:
  explicit TransformGuess(QWidget* parent = nullptr);
  ~TransformGuess();

  void configure(const YAML::Node& node) override;
  YAML::Node save() const override;

private:
  Ui::TransformGuess* ui_;
};

}  // namespace calibration_workbench
