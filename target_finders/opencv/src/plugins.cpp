#include <calibration_workbench/target_finders/opencv/aruco_grid_target_finder.h>
#include <calibration_workbench/target_finders/opencv/charuco_grid_target_finder.h>
#include <calibration_workbench/target_finders/opencv/chessboard_grid_target_finder.h>
#include <calibration_workbench/target_finders/opencv/modified_circle_grid_target_finder.h>

#include <boost_plugin_loader/macros.h>
#define EXPORT_TARGET_FINDER_PLUGIN(DERIVED_CLASS, ALIAS)                                                              \
  EXPORT_CLASS_SECTIONED(DERIVED_CLASS, ALIAS, TARGET_FINDER_SECTION)

EXPORT_TARGET_FINDER_PLUGIN(calibration_workbench::ArucoGridTargetFinderFactory, ArucoGridTargetFinder)
EXPORT_TARGET_FINDER_PLUGIN(calibration_workbench::CharucoGridTargetFinderFactory, CharucoGridTargetFinder)
EXPORT_TARGET_FINDER_PLUGIN(calibration_workbench::ChessboardGridTargetFinderFactory, ChessboardGridTargetFinder)
EXPORT_TARGET_FINDER_PLUGIN(calibration_workbench::ModifiedCircleGridTargetFinderFactory,
                            ModifiedCircleGridTargetFinder)
