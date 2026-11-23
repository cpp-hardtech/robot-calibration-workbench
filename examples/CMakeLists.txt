find_package(boost_plugin_loader REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(${PROJECT_NAME}_make_observations make_observations.cpp)
target_link_libraries(${PROJECT_NAME}_make_observations PRIVATE yaml-cpp)
target_cxx_version(${PROJECT_NAME}_make_observations PRIVATE VERSION 17)
install(TARGETS ${PROJECT_NAME}_make_observations RUNTIME DESTINATION lib/${PROJECT_NAME}
                                                          COMPONENT observation_manifest_cli)

if(CALIBRATION_WORKBENCH_ENABLE_TESTING)
  add_test(
    NAME MakeObservationsCli
    COMMAND
      ${CMAKE_COMMAND} "-DMAKE_OBSERVATIONS=$<TARGET_FILE:${PROJECT_NAME}_make_observations>"
      "-DTEST_DIRECTORY=${CMAKE_CURRENT_BINARY_DIR}/make-observations-test" -P
      "${CMAKE_CURRENT_SOURCE_DIR}/test_make_observations.cmake")
  add_dependencies(run_tests ${PROJECT_NAME}_make_observations)
endif()

# Camera intrinsic calibration example
add_executable(${PROJECT_NAME}_camera_intrinsic camera_intrinsic.cpp)
target_link_libraries(
  ${PROJECT_NAME}_camera_intrinsic
  PRIVATE ${PROJECT_NAME}_analysis
          ${PROJECT_NAME}_target_finders_opencv
          opencv_highgui
          stdc++fs
          boost_plugin_loader::boost_plugin_loader)
target_compile_definitions(${PROJECT_NAME}_camera_intrinsic PRIVATE EXAMPLE_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
target_cxx_version(${PROJECT_NAME}_camera_intrinsic PUBLIC VERSION 17)

# Hand-eye calibration example
add_executable(${PROJECT_NAME}_extrinsic_hand_eye extrinsic_hand_eye.cpp)
target_link_libraries(
  ${PROJECT_NAME}_extrinsic_hand_eye
  PRIVATE ${PROJECT_NAME}_analysis
          ${PROJECT_NAME}_target_finders_opencv
          opencv_highgui
          stdc++fs
          boost_plugin_loader::boost_plugin_loader)
target_compile_definitions(${PROJECT_NAME}_extrinsic_hand_eye
                           PRIVATE EXAMPLE_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
target_cxx_version(${PROJECT_NAME}_extrinsic_hand_eye PUBLIC VERSION 17)

# Kinematic calibration example
add_executable(${PROJECT_NAME}_kinematic_calibration kinematic_calibration.cpp)
target_link_libraries(${PROJECT_NAME}_kinematic_calibration PRIVATE ${PROJECT_NAME}_analysis
                                                                    ${PROJECT_NAME}_target_finders_opencv)
target_compile_definitions(${PROJECT_NAME}_kinematic_calibration
                           PRIVATE EXAMPLE_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
target_cxx_version(${PROJECT_NAME}_kinematic_calibration PUBLIC VERSION 17)

# Noise qualification example
add_executable(${PROJECT_NAME}_noise_qualification_2d noise_qualification_2d.cpp)
target_link_libraries(
  ${PROJECT_NAME}_noise_qualification_2d
  PRIVATE ${PROJECT_NAME}_analysis
          ${PROJECT_NAME}_target_finders_opencv
          opencv_highgui
          stdc++fs
          boost_plugin_loader::boost_plugin_loader)
target_compile_definitions(${PROJECT_NAME}_noise_qualification_2d
                           PRIVATE EXAMPLE_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
target_cxx_version(${PROJECT_NAME}_noise_qualification_2d PUBLIC VERSION 17)

# PnP example
add_executable(${PROJECT_NAME}_pnp pnp.cpp)
target_link_libraries(
  ${PROJECT_NAME}_pnp
  PRIVATE ${PROJECT_NAME}_analysis
          ${PROJECT_NAME}_target_finders_opencv
          opencv_highgui
          stdc++fs
          boost_plugin_loader::boost_plugin_loader)
target_compile_definitions(${PROJECT_NAME}_pnp PRIVATE EXAMPLE_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/data")
target_cxx_version(${PROJECT_NAME}_pnp PUBLIC VERSION 17)

add_executable(${PROJECT_NAME}_estimate_target_pose estimate_target_pose.cpp)
target_link_libraries(
  ${PROJECT_NAME}_estimate_target_pose
  PRIVATE ${PROJECT_NAME}_optimizations
          ${PROJECT_NAME}_target_finders_opencv
          opencv_calib3d
          boost_plugin_loader::boost_plugin_loader)
target_cxx_version(${PROJECT_NAME}_estimate_target_pose PRIVATE VERSION 17)
add_dependencies(${PROJECT_NAME}_estimate_target_pose ${PROJECT_NAME}_plugins_opencv)
install(TARGETS ${PROJECT_NAME}_estimate_target_pose RUNTIME DESTINATION lib/${PROJECT_NAME} COMPONENT target_pose_cli)

# Build unit tests from the examples
if(${CALIBRATION_WORKBENCH_ENABLE_TESTING})
  find_gtest()

  function(add_plugin_example_tests target)
    set(plugin_target ${PROJECT_NAME}_plugins_opencv)
    add_dependencies(${target} ${plugin_target})
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux"
       AND NOT
           CMAKE_VERSION
           VERSION_LESS
           "3.10")
      set(library_path "$<TARGET_FILE_DIR:${plugin_target}>")
      if(NOT
         "$ENV{LD_LIBRARY_PATH}"
         STREQUAL
         "")
        string(APPEND library_path ":$ENV{LD_LIBRARY_PATH}")
      endif()
      gtest_discover_tests(${target} DISCOVERY_MODE PRE_TEST PROPERTIES ENVIRONMENT "LD_LIBRARY_PATH=${library_path}")
    else()
      add_gtest_discover_tests(${target})
    endif()
  endfunction()

  add_test(
    NAME EstimateTargetPoseCli
    COMMAND
      ${CMAKE_COMMAND} "-DESTIMATE_POSE=$<TARGET_FILE:${PROJECT_NAME}_estimate_target_pose>"
      "-DPLUGIN_DIRECTORY=$<TARGET_FILE_DIR:${PROJECT_NAME}_plugins_opencv>"
      "-DSAMPLE_DIRECTORY=${CMAKE_CURRENT_SOURCE_DIR}/data/test_set_charuco"
      "-DFEW_CORNERS_IMAGE=${PROJECT_SOURCE_DIR}/test/images/charuco_one_corner.jpg"
      "-DTEST_DIRECTORY=${CMAKE_CURRENT_BINARY_DIR}/estimate-target-pose-test" -P
      "${CMAKE_CURRENT_SOURCE_DIR}/test_estimate_target_pose.cmake")
  add_dependencies(run_tests ${PROJECT_NAME}_estimate_target_pose)

  # Camera intrinsic calibration
  add_plugin_example_tests(${PROJECT_NAME}_camera_intrinsic)
  add_dependencies(run_tests ${PROJECT_NAME}_camera_intrinsic)
  target_link_libraries(${PROJECT_NAME}_camera_intrinsic PRIVATE GTest::GTest GTest::Main)
  target_compile_definitions(${PROJECT_NAME}_camera_intrinsic PUBLIC CALIBRATION_WORKBENCH_ENABLE_TESTING)

  # Hand-eye calibration
  add_plugin_example_tests(${PROJECT_NAME}_extrinsic_hand_eye)
  add_dependencies(run_tests ${PROJECT_NAME}_extrinsic_hand_eye)
  target_link_libraries(${PROJECT_NAME}_extrinsic_hand_eye PRIVATE GTest::GTest GTest::Main)
  target_compile_definitions(${PROJECT_NAME}_extrinsic_hand_eye PUBLIC CALIBRATION_WORKBENCH_ENABLE_TESTING)

  # Kinematic calibration
  add_gtest_discover_tests(${PROJECT_NAME}_kinematic_calibration)
  add_dependencies(run_tests ${PROJECT_NAME}_kinematic_calibration)
  target_link_libraries(${PROJECT_NAME}_kinematic_calibration PRIVATE GTest::GTest GTest::Main)
  target_compile_definitions(${PROJECT_NAME}_kinematic_calibration PUBLIC CALIBRATION_WORKBENCH_ENABLE_TESTING)

  # Noise qualification
  add_plugin_example_tests(${PROJECT_NAME}_noise_qualification_2d)
  add_dependencies(run_tests ${PROJECT_NAME}_noise_qualification_2d)
  target_link_libraries(${PROJECT_NAME}_noise_qualification_2d PRIVATE GTest::GTest GTest::Main)
  target_compile_definitions(${PROJECT_NAME}_noise_qualification_2d PUBLIC CALIBRATION_WORKBENCH_ENABLE_TESTING)

  # PnP
  add_plugin_example_tests(${PROJECT_NAME}_pnp)
  add_dependencies(run_tests ${PROJECT_NAME}_pnp)
  target_link_libraries(${PROJECT_NAME}_pnp PRIVATE GTest::GTest GTest::Main)
  target_compile_definitions(${PROJECT_NAME}_pnp PUBLIC CALIBRATION_WORKBENCH_ENABLE_TESTING)
endif()

add_executable(${PROJECT_NAME}_detect_target detect_target.cpp)
target_link_libraries(${PROJECT_NAME}_detect_target PRIVATE ${PROJECT_NAME}_target_finders_opencv
                                                            boost_plugin_loader::boost_plugin_loader)
target_cxx_version(${PROJECT_NAME}_detect_target PRIVATE VERSION 17)
add_dependencies(${PROJECT_NAME}_detect_target ${PROJECT_NAME}_plugins_opencv)

if(CALIBRATION_WORKBENCH_ENABLE_TESTING)
  add_test(
    NAME TargetDiagnosticsCli
    COMMAND
      ${CMAKE_COMMAND} "-DDETECT_TARGET=$<TARGET_FILE:${PROJECT_NAME}_detect_target>"
      "-DPLUGIN_DIRECTORY=$<TARGET_FILE_DIR:${PROJECT_NAME}_plugins_opencv>"
      "-DTARGET_CONFIG=${CMAKE_CURRENT_SOURCE_DIR}/data/test_set_charuco/cal_data.yaml"
      "-DTARGET_IMAGE=${PROJECT_SOURCE_DIR}/test/images/charuco_unobscured.jpg"
      "-DTEST_DIRECTORY=${CMAKE_CURRENT_BINARY_DIR}/target-diagnostics-test" -P
      "${CMAKE_CURRENT_SOURCE_DIR}/test_detect_target.cmake")
  add_dependencies(run_tests ${PROJECT_NAME}_detect_target)
endif()

# Install the data directory
install(DIRECTORY data DESTINATION share/${PROJECT_NAME})

list(APPEND SUPPORTED_COMPONENTS examples)
set(SUPPORTED_COMPONENTS "${SUPPORTED_COMPONENTS}" PARENT_SCOPE)

configure_component(
  COMPONENT
  examples
  NAMESPACE
  calibration_workbench
  TARGETS
  ${PROJECT_NAME}_camera_intrinsic
  ${PROJECT_NAME}_extrinsic_hand_eye
  ${PROJECT_NAME}_kinematic_calibration
  ${PROJECT_NAME}_noise_qualification_2d
  ${PROJECT_NAME}_pnp
  DEPENDENCIES
  "calibration_workbench COMPONENTS analysis target_finders_opencv")

# Install the executables for access by ROS
install(
  TARGETS ${PROJECT_NAME}_camera_intrinsic
          ${PROJECT_NAME}_extrinsic_hand_eye
          ${PROJECT_NAME}_kinematic_calibration
          ${PROJECT_NAME}_noise_qualification_2d
          ${PROJECT_NAME}_pnp
          ${PROJECT_NAME}_detect_target
  DESTINATION lib/${PROJECT_NAME})
