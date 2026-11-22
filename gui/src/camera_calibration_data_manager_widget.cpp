#include "ui_camera_calibration_data_manager_widget.h"

#include <calibration_workbench/gui/camera_calibration_data_manager_widget.h>
#include <calibration_workbench/gui/target_finder.h>
#include <calibration_workbench/gui/camera_intrinsics.h>
#include <calibration_workbench/gui/aspect_ratio_pixmap_label.h>
#include <calibration_workbench/target_finders/opencv/utils.h>
#include <calibration_workbench/core/serialization.h>
// Analysis
#include <calibration_workbench/analysis/homography_analysis.h>

#include <boost_plugin_loader/plugin_loader.hpp>
#include <QAction>
#include <QCloseEvent>
#include <QDialog>
#include <QFileDialog>
#include <QFile>
#include <QMessageBox>
#include <QPixmap>
#include <QScrollBar>
#include <QSaveFile>
#include <QTextStream>
#include <QVBoxLayout>
#include <memory>
#include <utility>
#include <vector>

namespace calibration_workbench
{
const unsigned CameraCalibrationDataManagerWidget::RANDOM_SEED = 1;
const int CameraCalibrationDataManagerWidget::IMAGE_FILE_NAME_ROLE = Qt::UserRole + 1;
const int CameraCalibrationDataManagerWidget::POSE_FILE_NAME_ROLE = Qt::UserRole + 2;
const int CameraCalibrationDataManagerWidget::IDX_FEATURES = 0;
const int CameraCalibrationDataManagerWidget::IDX_HOMOGRAPHY = 1;

QPixmap CameraCalibrationDataManagerWidget::toQt(const cv::Mat& image)
{
  return QPixmap::fromImage(QImage(image.data, image.cols, image.rows, image.step, QImage::Format_RGB888).rgbSwapped());
}

void CameraCalibrationDataManagerWidget::info(QTreeWidgetItem* item, const QString& message)
{
  item->setText(1, message);
  item->setForeground(1, QApplication::palette().text());
}

void CameraCalibrationDataManagerWidget::error(QTreeWidgetItem* item, const QString& message)
{
  item->setText(1, message);
  item->setForeground(1, QBrush(QColor("red")));
}

CameraCalibrationDataManagerWidget::CameraCalibrationDataManagerWidget(QWidget* parent)
  : QWidget(parent)
  , ui_(new Ui::CameraCalibrationDataManager())
  , target_finder_widget_(new TargetFinderWidget(this))
  , camera_intrinsics_widget_(new CameraIntrinsicsWidget(this))
  , action_load_observations(new QAction("Load observations...", this))
  , action_edit_target_finder(new QAction("Target finder", this))
  , action_edit_camera_intrinsics(new QAction("Camera intrinsics", this))
{
  ui_->setupUi(this);

  // Configure the tree widget
  ui_->tree_widget_observations->setEditTriggers(QAbstractItemView::NoEditTriggers);

  // Configure the actions
  action_load_observations->setToolTip("Load observations file...");
  action_load_observations->setIcon(QIcon::fromTheme("document-open"));
  action_edit_target_finder->setToolTip("Edit target finder configuration");
  action_edit_target_finder->setIcon(QIcon::fromTheme("edit-find"));
  action_edit_camera_intrinsics->setToolTip("Edit camera intrinsics");
  action_edit_camera_intrinsics->setIcon(QIcon::fromTheme("camera-photo"));

  // Set up the target finder widget
  {
    auto dialog = new QDialog(this);
    auto layout = new QVBoxLayout(dialog);
    layout->addWidget(target_finder_widget_);
    dialog->setWindowTitle("Edit target finder");

    connect(action_edit_target_finder, &QAction::triggered, dialog, &QWidget::show);
  }

  // Set up the camera intrinsics widget
  {
    auto dialog = new QDialog(this);
    auto layout = new QVBoxLayout(dialog);
    layout->addWidget(camera_intrinsics_widget_);
    dialog->setWindowTitle("Edit camera intrinsics");

    connect(action_edit_camera_intrinsics, &QAction::triggered, dialog, &QWidget::show);
  }

  // Move the text edit scroll bar to the maximum limit whenever it is resized
  connect(ui_->text_edit_results->verticalScrollBar(), &QScrollBar::rangeChanged, [this]() {
    ui_->text_edit_results->verticalScrollBar()->setSliderPosition(
        ui_->text_edit_results->verticalScrollBar()->maximum());
  });

  // Set up push buttons
  connect(action_load_observations, &QAction::triggered, this, &CameraCalibrationDataManagerWidget::onLoadObservations);
  connect(ui_->tree_widget_observations, &QTreeWidget::itemClicked, this,
          &CameraCalibrationDataManagerWidget::drawImage);

  // Set up the plugin loader
  loader_.search_libraries.insert(loader_.search_libraries.end(), CALIBRATION_WORKBENCH_PLUGIN_LIBRARIES);
  loader_.search_libraries_env = CALIBRATION_WORKBENCH_SEARCH_LIBRARIES_ENV;
}

CameraCalibrationDataManagerWidget::~CameraCalibrationDataManagerWidget() { delete ui_; }

QTreeWidget* CameraCalibrationDataManagerWidget::getTreeWidget() const { return ui_->tree_widget_observations; }

void CameraCalibrationDataManagerWidget::closeEvent(QCloseEvent* event)
{
  QMessageBox::StandardButton ret = QMessageBox::question(this, "Exit", "Are you sure you want to exit?");
  switch (ret)
  {
    case QMessageBox::StandardButton::No:
      event->ignore();
      break;
    default:
      event->accept();
  }
}

void CameraCalibrationDataManagerWidget::loadTargetFinder()
{
  // Get target type and currentconfig
  YAML::Node target_finder_config = target_finder_widget_->save();
  auto target_type = getMember<std::string>(target_finder_config, "type");

  factory_ = loader_.createInstance<TargetFinderFactoryOpenCV>(target_type);
  target_finder_ = factory_->create(target_finder_config);
}

void CameraCalibrationDataManagerWidget::onLoadObservations()
{
  try
  {
    QString observations_file =
        QFileDialog::getOpenFileName(this, "Load calibration observation file", QString(), "YAML files (*.yaml *.yml)");
    if (observations_file.isNull() || observations_file.isEmpty())
      return;

    loadObservations(observations_file.toStdString());
  }
  catch (const std::exception& ex)
  {
    QMessageBox::warning(this, "Error", ex.what());
  }
}

void CameraCalibrationDataManagerWidget::loadObservations(const std::string& observations_file)
{
  QFileInfo observations_file_info(QString::fromStdString(observations_file));

  const auto observations = getMember<YAML::Node>(YAML::LoadFile(observations_file), "data");
  if (!observations.IsSequence())
    throw ICalException("Observation data must be a sequence");

  std::vector<std::unique_ptr<QTreeWidgetItem>> items;
  items.reserve(observations.size());

  for (std::size_t i = 0; i < observations.size(); ++i)
  {
    const YAML::Node& entry = observations[i];

    QString image_file =
        observations_file_info.absoluteDir().filePath(QString::fromStdString(getMember<std::string>(entry, "image")));
    QString pose_file =
        observations_file_info.absoluteDir().filePath(QString::fromStdString(getMember<std::string>(entry, "pose")));

    auto item = std::make_unique<QTreeWidgetItem>();
    item->setData(0, Qt::EditRole, "Observation " + QString::number(i));
    item->setData(0, IMAGE_FILE_NAME_ROLE, image_file);
    item->setData(0, POSE_FILE_NAME_ROLE, pose_file);

    // Add a column entry for the number of detected features
    auto features_item = new QTreeWidgetItem(item.get());
    features_item->setData(0, Qt::EditRole, "Feature count");
    info(features_item, "-");

    // Add a column entry for the homography error
    auto homography_item = new QTreeWidgetItem(item.get());
    homography_item->setData(0, Qt::EditRole, "Homography error (px)");
    info(homography_item, "-");

    items.push_back(std::move(item));
  }

  ui_->tree_widget_observations->clear();
  for (auto& item : items)
    ui_->tree_widget_observations->addTopLevelItem(item.release());

  ui_->tree_widget_observations->resizeColumnToContents(0);
}

QAction* CameraCalibrationDataManagerWidget::createObservationReportAction()
{
  auto action = new QAction("Export observation report...", this);
  connect(action, &QAction::triggered, this, &CameraCalibrationDataManagerWidget::onSaveObservationReport);
  return action;
}

void CameraCalibrationDataManagerWidget::onSaveObservationReport()
{
  try
  {
    const QString file =
        QFileDialog::getSaveFileName(this, "Export observation report", QString(), "CSV files (*.csv)");
    if (!file.isEmpty())
      saveObservationReport(file.toStdString());
  }
  catch (const std::exception& ex)
  {
    QMessageBox::warning(this, "Error", ex.what());
  }
}

void CameraCalibrationDataManagerWidget::saveObservationReport(const std::string& file) const
{
  const auto tree = ui_->tree_widget_observations;
  if (tree->topLevelItemCount() == 0)
    throw ICalException("No observations have been added");

  QSaveFile output(QString::fromStdString(file));
  if (!output.open(QIODevice::WriteOnly | QIODevice::Text))
    throw ICalException("Failed to open observation report: '" + file + "'");

  const auto escape = [](QString field) {
    field.replace(QLatin1Char('"'), QStringLiteral("\"\""));
    return QLatin1Char('"') + field + QLatin1Char('"');
  };

  QTextStream stream(&output);
  stream.setCodec("UTF-8");
  stream << "image_path,pose_path,feature_count,homography_error_px,status\n";
  for (int i = 0; i < tree->topLevelItemCount(); ++i)
  {
    const auto item = tree->topLevelItem(i);
    const QString feature_count = item->child(IDX_FEATURES)->text(1);
    const QString homography_error = item->child(IDX_HOMOGRAPHY)->text(1);
    stream << escape(item->data(0, IMAGE_FILE_NAME_ROLE).toString()) << ','
           << escape(item->data(0, POSE_FILE_NAME_ROLE).toString()) << ','
           << (feature_count == "-" ? QString() : feature_count) << ','
           << (homography_error == "-" ? QString() : homography_error) << ',' << escape(item->text(1)) << '\n';
  }

  stream.flush();
  if (stream.status() != QTextStream::Ok || !output.commit())
    throw ICalException("Failed to write observation report: '" + file + "'");
}

void CameraCalibrationDataManagerWidget::drawImage(QTreeWidgetItem* item, int col)
{
  if (item == nullptr)
    return;

  // Extract the top level item
  while (item->parent() != nullptr)
    item = item->parent();

  QTreeWidgetItem* features_item = item->child(IDX_FEATURES);
  QTreeWidgetItem* homography_item = item->child(IDX_HOMOGRAPHY);

  QString image_file = item->data(0, IMAGE_FILE_NAME_ROLE).value<QString>();
  if (!QFile(image_file).exists())
  {
    error(item, "Image file does not exist");
    return;
  }

  cv::Mat image;
  try
  {
    image = readImageOpenCV(image_file.toStdString());
    loadTargetFinder();

    // Detect the target in the image
    TargetFeatures2D features = target_finder_->findTargetFeatures(image);
    cv::Mat detected_image = target_finder_->drawTargetFeatures(image, features);
    info(item, "Successfully identified target");

    // Save the number of detected features to the tree
    info(features_item, QString::number(features.size()));

    // Emit the image
    emit imageSelected(toQt(detected_image));

    // Compute the homography error
    Correspondence2D3D::Set corrs = target_finder_->target().createCorrespondences(features);
    RandomCorrespondenceSampler random_sampler(corrs.size(), corrs.size() / 3, RANDOM_SEED);
    Eigen::VectorXd homography_error = calculateHomographyError(corrs, random_sampler);
    double homography_error_mean = homography_error.array().mean();

    // Save the homography error to the tree
    info(homography_item, QString::number(homography_error_mean));

    // Check homography threshold and update notes/row color
    if (homography_error_mean > ui_->double_spin_box_homography_threshold->value())
      error(item, "Homography threshold violated");
  }
  catch (const std::exception& ex)
  {
    if (!image.empty())
      emit imageSelected(toQt(image));

    error(item, QString(ex.what()));
    info(features_item, "-");
    info(homography_item, "-");
  }
}

}  // namespace calibration_workbench
