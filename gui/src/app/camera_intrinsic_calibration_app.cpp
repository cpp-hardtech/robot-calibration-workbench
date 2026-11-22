#include <calibration_workbench/gui/camera_intrinsic_calibration_widget.h>
#include <calibration_workbench/gui/aspect_ratio_pixmap_label.h>

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
#include <QTextStream>
#include <QToolBar>
#include <signal.h>

void handleSignal(int /*sig*/) { QApplication::instance()->quit(); }

int main(int argc, char** argv)
{
  const bool headless = argc > 1 && std::string_view(argv[1]) == "--headless";
  if (headless && (argc < 5 || argc > 6))
  {
    std::cerr << "Usage: calibration_workbench_camera_intrinsic_calibration_app --headless config.yaml observations.yaml results.yaml [ros-results.yaml]\n";
    return 1;
  }

  int qt_argc = 1;
  char* qt_argv[] = {argv[0], nullptr};
  if (headless)
    qputenv("QT_QPA_PLATFORM", "offscreen");
  QApplication app(headless ? qt_argc : argc, headless ? qt_argv : argv);

  signal(SIGINT, handleSignal);
  signal(SIGTERM, handleSignal);

  // Create the main window
  auto window = new QMainWindow();
  window->setWindowTitle("Camera Intrinsic Calibration");
  window->setWindowIcon(QIcon(":/icons/icon.jpg"));
  window->setStyleSheet(QString("QMainWindow::separator { background-color: %1; width: 3px; height: 3px; }")
                            .arg(app.palette().color(QPalette::Midlight).name()));

  // Create the calibration widget
  auto cal = new calibration_workbench::CameraIntrinsicCalibrationWidget(window);
  cal->action_use_extrinsic_guesses->setChecked(true);

  // Add the calibration widget as a docked widget on the left side of the main window
  auto dock = new QDockWidget("Calibration", window);
  dock->setStyleSheet(
      QString("QDockWidget::title { background-color: %1; }").arg(app.palette().color(QPalette::Midlight).name()));
  dock->setFeatures(QDockWidget::DockWidgetFloatable | QDockWidget::DockWidgetMovable);
  dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
  dock->setWidget(cal);
  window->addDockWidget(Qt::LeftDockWidgetArea, dock);

  // Create a label widget for displaying the images
  auto image = new calibration_workbench::AspectRatioPixmapLabel(window);
  image->setText(QString::fromStdString(cal->getInstructions()));
  image->setAlignment(Qt::AlignCenter);

  // Set the label as the central widget
  window->setCentralWidget(image);

  // Connect the calibration image selected signal to the label set pixmap slot
  QApplication::connect(cal, &calibration_workbench::CameraIntrinsicCalibrationWidget::imageSelected, image,
                        &calibration_workbench::AspectRatioPixmapLabel::setPixmap);

  // Set up the tool bar
  window->addToolBar(cal->tool_bar);

  // Set up the menu bar
  {
    QMenu* menu_file = window->menuBar()->addMenu("File");
    menu_file->addAction(cal->action_load_observations);
    menu_file->addAction(cal->action_load_configuration);
    menu_file->addSeparator();
    menu_file->addAction(cal->action_save);
    menu_file->addAction(cal->action_save_ros_format);
  }
  {
    QMenu* menu_edit = window->menuBar()->addMenu("Edit");
    menu_edit->addAction(cal->action_edit_target_finder);
    menu_edit->addAction(cal->action_edit_camera_intrinsics);
    menu_edit->addAction(cal->action_use_extrinsic_guesses);
    menu_edit->addAction(cal->action_use_opencv);
  }
  {
    QMenu* menu_calibrate = window->menuBar()->addMenu("Calibrate");
    menu_calibrate->addAction(cal->action_calibrate);
  }
  {
    QMenu* menu_help = window->menuBar()->addMenu("Help");
    menu_help->addAction(cal->action_instructions);
  }

  /* Attempt to run headless if all files are specified:
   *   argv[1]: configuration file
   *   argv[2]: observation file
   *   argv[3]: results file (calibration_workbench)
   *   argv[4]: results file (ROS) - optional
   */
  if (argc > 3)
  {
    try
    {
      const int first_file = headless ? 2 : 1;
      cal->loadConfig(argv[first_file]);
      cal->loadObservations(argv[first_file + 1]);
      cal->calibrate();
      if (headless && !cal->calibrationConverged())
      {
        std::cerr << "Calibration failed to converge\n";
        return 1;
      }
      cal->saveResults(argv[first_file + 2]);
      if (argc > first_file + 3)
        cal->saveROSFormat(argv[first_file + 3]);

      if (headless)
        return 0;

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
        window->showMaximized();
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
