#include "ui_modified_circle_grid_target_finder.h"
#include <calibration_workbench/gui/modified_circle_grid_target_finder.h>
#include <calibration_workbench/target_finders/opencv/circle_detector.h>
#include <algorithm>
#include <cstddef>

namespace calibration_workbench
{
ModifiedCircleGridTargetFinderWidget::ModifiedCircleGridTargetFinderWidget(QWidget* parent)
  : ConfigurableWidget(parent), ui_(new Ui::ModifiedCircleGridTargetFinder)
{
  ui_->setupUi(this);

  ui_->frame_filter_area->setEnabled(ui_->filterByAreaCheckBox->isChecked());
  ui_->frame_filter_circularity->setEnabled(ui_->filterByCircularityCheckBox->isChecked());
  ui_->frame_filter_color->setEnabled(ui_->filterByColorCheckBox->isChecked());
  ui_->frame_filter_convexity->setEnabled(ui_->filterByConvexityCheckBox->isChecked());
  ui_->frame_filter_inertia->setEnabled(ui_->filterByInertiaCheckBox->isChecked());

  connect(ui_->filterByAreaCheckBox, &QCheckBox::clicked, ui_->frame_filter_area, &QGroupBox::setEnabled);
  connect(ui_->filterByCircularityCheckBox, &QCheckBox::clicked, ui_->frame_filter_circularity, &QGroupBox::setEnabled);
  connect(ui_->filterByColorCheckBox, &QCheckBox::clicked, ui_->frame_filter_color, &QGroupBox::setEnabled);
  connect(ui_->filterByConvexityCheckBox, &QCheckBox::clicked, ui_->frame_filter_convexity, &QGroupBox::setEnabled);
  connect(ui_->filterByInertiaCheckBox, &QCheckBox::clicked, ui_->frame_filter_inertia, &QGroupBox::setEnabled);
}

ModifiedCircleGridTargetFinderWidget::~ModifiedCircleGridTargetFinderWidget() { delete ui_; }

void ModifiedCircleGridTargetFinderWidget::configure(const YAML::Node& node)
{
  ui_->rowSpinBox->setValue(node["rows"].as<int>());
  ui_->colSpinBox->setValue(node["cols"].as<int>());
  ui_->spacingDoubleSpinBox->setValue(node["spacing"].as<double>());

  const auto detector = node["circle_detector_params"];
  const auto params = detector ? detector.as<CircleDetectorParams>() : CircleDetectorParams{};
  const auto set_count = [](QSpinBox* box, std::size_t count) {
    box->setValue(static_cast<int>(std::min(count, static_cast<std::size_t>(box->maximum()))));
  };

  ui_->minThresholdSpinBox->setValue(params.minThreshold);
  ui_->maxThresholdSpinBox->setValue(params.maxThreshold);
  set_count(ui_->numThresholdSpinBox, params.nThresholds);
  set_count(ui_->minRepeatSpinBox, params.minRepeatability);
  ui_->circleInclusionRadiusDoubleSpinBox->setValue(params.circleInclusionRadius);
  ui_->maxRadiusDiffDoubleSpinBox->setValue(params.maxRadiusDiff);
  ui_->maxAvgEllipseErrorDoubleSpinBox->setValue(params.maxAverageEllipseError);

  ui_->filterByColorCheckBox->setChecked(params.filterByColor);
  ui_->frame_filter_color->setEnabled(params.filterByColor);
  ui_->circleColorSpinBox->setValue(params.circleColor);

  ui_->filterByAreaCheckBox->setChecked(params.filterByArea);
  ui_->frame_filter_area->setEnabled(params.filterByArea);
  ui_->minAreaDoubleSpinBox->setValue(params.minArea);
  ui_->maxAreaDoubleSpinBox->setValue(params.maxArea);

  ui_->filterByCircularityCheckBox->setChecked(params.filterByCircularity);
  ui_->frame_filter_circularity->setEnabled(params.filterByCircularity);
  ui_->minCircularityDoubleSpinBox->setValue(params.minCircularity);
  ui_->maxCircularityDoubleSpinBox->setValue(params.maxCircularity);

  ui_->filterByInertiaCheckBox->setChecked(params.filterByInertia);
  ui_->frame_filter_inertia->setEnabled(params.filterByInertia);
  ui_->minInertiaRatioDoubleSpinBox->setValue(params.minInertiaRatio);
  ui_->maxInertiaRatioDoubleSpinBox->setValue(params.maxInertiaRatio);

  ui_->filterByConvexityCheckBox->setChecked(params.filterByConvexity);
  ui_->frame_filter_convexity->setEnabled(params.filterByConvexity);
  ui_->minConvexityDoubleSpinBox->setValue(params.minConvexity);
  ui_->maxConvexityDoubleSpinBox->setValue(params.maxConvexity);
}

YAML::Node ModifiedCircleGridTargetFinderWidget::save() const
{
  YAML::Node node;
  node["type"] = "ModifiedCircleGridTargetFinder";

  node["rows"] = ui_->rowSpinBox->value();
  node["cols"] = ui_->colSpinBox->value();
  node["spacing"] = ui_->spacingDoubleSpinBox->value();

  YAML::Node subnode;
  node["circle_detector_params"] = subnode;

  subnode["minThreshold"] = ui_->minThresholdSpinBox->value();
  subnode["maxThreshold"] = ui_->maxThresholdSpinBox->value();
  subnode["nThresholds"] = ui_->numThresholdSpinBox->value();

  subnode["minRepeatability"] = ui_->minRepeatSpinBox->value();
  subnode["circleInclusionRadius"] = ui_->circleInclusionRadiusDoubleSpinBox->value();
  subnode["maxRadiusDiff"] = ui_->maxRadiusDiffDoubleSpinBox->value();

  subnode["maxAverageEllipseError"] = ui_->maxAvgEllipseErrorDoubleSpinBox->value();

  subnode["filterByColor"] = ui_->filterByColorCheckBox->isChecked();
  subnode["circleColor"] = ui_->circleColorSpinBox->value();

  subnode["filterByArea"] = ui_->filterByAreaCheckBox->isChecked();
  subnode["minArea"] = ui_->minAreaDoubleSpinBox->value();
  subnode["maxArea"] = ui_->maxAreaDoubleSpinBox->value();

  subnode["filterByCircularity"] = ui_->filterByCircularityCheckBox->isChecked();
  subnode["minCircularity"] = ui_->minCircularityDoubleSpinBox->value();
  subnode["maxCircularity"] = ui_->maxCircularityDoubleSpinBox->value();

  subnode["filterByInertia"] = ui_->filterByInertiaCheckBox->isChecked();
  subnode["minInertiaRatio"] = ui_->minInertiaRatioDoubleSpinBox->value();
  subnode["maxInertiaRatio"] = ui_->maxInertiaRatioDoubleSpinBox->value();

  subnode["filterByConvexity"] = ui_->filterByConvexityCheckBox->isChecked();
  subnode["minConvexity"] = ui_->minConvexityDoubleSpinBox->value();
  subnode["maxConvexity"] = ui_->maxConvexityDoubleSpinBox->value();

  return node;
}

}  // namespace calibration_workbench
