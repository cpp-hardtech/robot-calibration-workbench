# OpenCV
option(CALIBRATION_WORKBENCH_BUILD_OPENCV "Build OpenCV target finders" ON)
if(CALIBRATION_WORKBENCH_BUILD_OPENCV)
  message("Building OpenCV target finders")
  list(APPEND SUPPORTED_COMPONENTS target_finders_opencv)
  set(SUPPORTED_COMPONENTS "${SUPPORTED_COMPONENTS}" PARENT_SCOPE)
  add_subdirectory(opencv)
endif()
