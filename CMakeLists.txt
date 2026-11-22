cmake_minimum_required(VERSION 3.5.1)

# Build helpers are vendored under cmake/vendor/. See cmake/vendor/README.md.
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/vendor")
include(extract_package_metadata)
include(cmake_tools)
include(code_coverage)

# Extract package name and version from package.xml
extract_package_metadata(pkg)
project(${pkg_extracted_name} VERSION ${pkg_extracted_version} LANGUAGES CXX)

set(SUPPORTED_COMPONENTS "")

add_subdirectory(core)
add_subdirectory(optimizations)
add_subdirectory(analysis)
add_subdirectory(target_finders)

# GUI
option(CALIBRATION_WORKBENCH_BUILD_GUI "Build GUI" ON)
if(CALIBRATION_WORKBENCH_BUILD_GUI)
  message("Building GUI")
  add_subdirectory(gui)
endif()

# Unit tests
if(${CALIBRATION_WORKBENCH_ENABLE_TESTING})
  enable_testing()
  add_run_tests_target(ENABLE ${CALIBRATION_WORKBENCH_ENABLE_RUN_TESTING})
  add_subdirectory(test)
endif()

# Examples
option(CALIBRATION_WORKBENCH_BUILD_EXAMPLES "Build examples" ON)
if(CALIBRATION_WORKBENCH_BUILD_EXAMPLES)
  message("Building examples")
  add_subdirectory(examples)
endif()

# Package configuration
configure_package(SUPPORTED_COMPONENTS ${SUPPORTED_COMPONENTS})
