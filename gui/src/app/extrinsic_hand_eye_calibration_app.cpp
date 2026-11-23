#include <calibration_workbench/gui/extrinsic_hand_eye_calibration_widget.h>
#include <calibration_workbench/gui/aspect_ratio_pixmap_label.h>

#include "headless_calibration.h"

#include "headless_result_paths.h"

#include <filesystem>
#include <iostream>
#include <string_view>
#include <QApplication>
#include <QDockWidget>
#include <QIcon>
#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPushButton>
#include <QTextStream>
#include <QToolBar>
#include <signal.h>

void handleSignal(int /*sig*/) { QApplication::instance()->quit(); }

int main(int argc, char** argv)
{
  const std::string_view mode = argc > 1 ? argv[1] : "";
  const bool headless = mode == "--headless";
  const bool validation_only = mode == "--validate-only";
  if (validation_only && (argc != 5 || argv[4][0] == '\0'))
  {
    std::cerr << "Usage: " << argv[0] << " --validate-only config.yaml observations.yaml report.csv\n";
    return 1;
  }

  const auto output = headless ? calibration_workbench::parseHeadlessOutputArguments(argc, argv, 5) : std::nullopt;
  if (headless && !output)
  {
    std::cerr << "Usage: calibration_workbench_extrinsic_hand_eye_calibration_app --headless config.yaml "
                 "observations.yaml results.yaml [--observation-report report.csv]\n";
    return 1;
  }

  int qt_argc = 1;
  char* qt_argv[] = { argv[0], nullptr };
  const bool non_interactive = headless || validation_only;
  if (non_interactive)
    qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(non_interactive ? qt_argc : argc, non_interactive ? qt_argv : argv);

  signal(SIGINT, handleSignal);
  signal(SIGTERM, handleSignal);

  // Create the main window
  auto window = new QMainWindow();
  window->setWindowTitle("Extrinsic Hand Eye Calibration");
  window->setWindowIcon(QIcon(":/icons/icon.jpg"));
  window->setStyleSheet(QString("QMainWindow::separator { background-color: %1; width: 3px; height: 3px; }")
                            .arg(app.palette().color(QPalette::Midlight).name()));

  // Create the calibration widget
  auto cal = new calibration_workbench::ExtrinsicHandEyeCalibrationWidget(window);

  if (validation_only)
    return calibration_workbench::runObservationValidation(*cal, argv);

  // Add the calibration widget as a docked widget on the left side of the main window
  auto dock = new QDockWidget("Calibration", window);
  dock->setStyleSheet(
      QString("QDockWidget::title { background-color: %1; }").arg(app.palette().color(QPalette::Midlight).name()));
  dock->setFeatures(QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable);
  dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  dock->setWidget(cal);
  window->addDockWidget(Qt::LeftDockWidgetArea, dock);

  // Create a label widget to display images
  auto image = new calibration_workbench::AspectRatioPixmapLabel(window);
  image->setText(QString::fromStdString(cal->getInstructions()));
  image->setAlignment(Qt::AlignCenter);

  // Set the label widget as the central widget in the main window
  window->setCentralWidget(image);

  // Connect the calibration image selected signal to the image pixmap slot
  QApplication::connect(cal, &calibration_workbench::ExtrinsicHandEyeCalibrationWidget::imageSelected, image,
                        &calibration_workbench::AspectRatioPixmapLabel::setPixmap);

  // Set up the tool bar
  window->addToolBar(cal->tool_bar);

  // Set up the menu bar
  {
    QMenu* menu_file = window->menuBar()->addMenu("File");
    menu_file->addAction(cal->action_load_observations);
    menu_file->addAction(cal->action_load_configuration);
    menu_file->addAction(cal->action_save);
    menu_file->addAction(cal->createObservationReportAction());
  }
  {
    QMenu* menu_edit = window->menuBar()->addMenu("Edit");
    menu_edit->addAction(cal->action_edit_target_finder);
    menu_edit->addAction(cal->action_edit_camera_intrinsics);
    menu_edit->addAction(cal->action_camera_mount_to_camera);
    menu_edit->addAction(cal->action_target_mount_to_target);
    menu_edit->addAction(cal->action_static_camera);
  }
  {
    QMenu* menu_calibrate = window->menuBar()->addMenu("Calibrate");
    menu_calibrate->addAction(cal->action_calibrate);
  }
  {
    QMenu* menu_help = window->menuBar()->addMenu("Help");
    menu_help->addAction(cal->action_instructions);
  }

  // Attempt to run headless if the configuration file (argv[1]), observation file (argv[2]), and results file (argv[3])
  // are specified
  if (argc > 3)
  {
    try
    {
      const int first_file = headless ? 2 : 1;
      if (headless)
        calibration_workbench::validateHeadlessResultPaths(argv[first_file], argv[first_file + 1],
                                                           argv[first_file + 2]);
      if (output)
        calibration_workbench::validateObservationReportPath(output->observation_report, argv + 2,
                                                             argv + output->positional_argc);
      cal->loadConfig(argv[first_file]);
      cal->loadObservations(argv[first_file + 1]);
      if (output)
        cal->validateObservationOutputPaths(argv[first_file + 2], nullptr, output->observation_report);
      cal->calibrate();
      if (headless && !cal->calibrationConverged())
      {
        std::cerr << "Calibration failed to converge\n";
        return 1;
      }
      cal->saveResults(argv[first_file + 2]);
      if (output)
      {
        if (output->observation_report)
          cal->saveObservationReport(output->observation_report);
        return 0;
      }

      QMessageBox::StandardButton ret = QMessageBox::question(nullptr, "Calibration",
                                                              "Successfully completed calibration and saved results. "
                                                              "View results in the GUI?");
      if (ret == QMessageBox::StandardButton::Yes)
      {
        window->showMaximized();
        return app.exec();
      }
    }
    catch (const std::exception& ex)
    {
      if (headless)
      {
        std::cerr << "Calibration error: " << ex.what() << '\n';
        return 1;
      }

      QString question;
      QTextStream ss(&question);
      ss << "Error: " << ex.what() << "\n\nOpen GUI to fix?";

      QMessageBox::StandardButton ret = QMessageBox::question(nullptr, "Error", question);
      if (ret == QMessageBox::StandardButton::Yes)
      {
        window->show();
        return app.exec();
      }
    }
  }
  else
  {
    window->showMaximized();

    // Attempt to load configuration and observations files if available
    try
    {
      if (argc > 1 && std::filesystem::is_regular_file(argv[1]))
      {
        cal->loadConfig(argv[1]);
        QMessageBox::information(nullptr, "Configuration", "Successfully loaded calibration configuration");
      }
      if (argc > 2 && std::filesystem::is_regular_file(argv[2]))
        cal->loadObservations(argv[2]);
    }
    catch (const std::exception& ex)
    {
      QMessageBox::warning(nullptr, "Error", ex.what());
    }

    return app.exec();
  }

  return 0;
}
