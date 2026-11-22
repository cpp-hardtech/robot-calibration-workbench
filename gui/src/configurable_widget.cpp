#include <calibration_workbench/gui/configurable_widget.h>
#include <QVBoxLayout>

namespace calibration_workbench
{
ConfigurableWidget::ConfigurableWidget(QWidget* parent) : QWidget(parent) {}

ConfigurableWidgetDialog::ConfigurableWidgetDialog(ConfigurableWidget* widget_, QWidget* parent)
  : QDialog(parent), widget(widget_)
{
  auto* vl = new QVBoxLayout(this);
  vl->addWidget(widget);
  setWindowTitle(" ");
}

}  // namespace calibration_workbench
