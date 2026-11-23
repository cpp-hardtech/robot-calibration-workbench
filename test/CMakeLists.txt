find_gtest()

# Build the test support library
add_library(
  ${PROJECT_NAME}_test_support SHARED
  src/pose_generator.cpp
  src/observation_creator.cpp
  src/dh_chain_observation_creator.cpp
  src/utilities.cpp
  src/circle_fit.cpp)
target_link_libraries(${PROJECT_NAME}_test_support PUBLIC ${PROJECT_NAME}_optimizations)
target_include_directories(${PROJECT_NAME}_test_support PUBLIC include)
target_compile_definitions(${PROJECT_NAME}_test_support PUBLIC RANDOM_SEED=0)

# The actual tests...

if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND TARGET ${PROJECT_NAME}_camera_intrinsic_calibration_app)
  set(intrinsic_app ${PROJECT_NAME}_camera_intrinsic_calibration_app)
  set(rms_check ${PROJECT_NAME}_intrinsic_reprojection_rms_check)
  set(plugin_target ${PROJECT_NAME}_plugins_opencv)
  add_executable(${rms_check} intrinsic_reprojection_rms_check.cpp)
  target_link_libraries(${rms_check} PRIVATE ${PROJECT_NAME}_optimizations)
  add_dependencies(
    run_tests
    ${intrinsic_app}
    ${rms_check}
    ${plugin_target})
  add_test(
    NAME HeadlessCalibration.IntrinsicReprojectionRMS
    COMMAND
      "${CMAKE_COMMAND}" "-DINTRINSIC_APP=$<TARGET_FILE:${intrinsic_app}>" "-DRMS_CHECK=$<TARGET_FILE:${rms_check}>"
      "-DPLUGIN_DIR=$<TARGET_FILE_DIR:${plugin_target}>"
      "-DSAMPLE_DIR=${PROJECT_SOURCE_DIR}/examples/data/test_set_10x10"
      "-DOUTPUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/intrinsic_reprojection_rms" -P
      "${CMAKE_CURRENT_SOURCE_DIR}/intrinsic_reprojection_rms.cmake")
  set_tests_properties(HeadlessCalibration.IntrinsicReprojectionRMS PROPERTIES TIMEOUT 30)
endif()

# Conversions
add_executable(${PROJECT_NAME}_conversion_tests conversion_utest.cpp)
target_link_libraries(${PROJECT_NAME}_conversion_tests PRIVATE ${PROJECT_NAME}_test_support GTest::GTest GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_conversion_tests)
add_dependencies(run_tests ${PROJECT_NAME}_conversion_tests)

# Extrinsic multi-static camera
add_executable(${PROJECT_NAME}_extrinsic_multi_static_camera_tests extrinsic_multi_static_camera_utest.cpp)
target_link_libraries(${PROJECT_NAME}_extrinsic_multi_static_camera_tests PRIVATE ${PROJECT_NAME}_test_support
                                                                                  GTest::GTest GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_extrinsic_multi_static_camera_tests)
add_dependencies(run_tests ${PROJECT_NAME}_extrinsic_multi_static_camera_tests)

# Extrinsic hand-eye
add_executable(${PROJECT_NAME}_extrinsic_hand_eye_tests extrinsic_hand_eye_utest.cpp)
target_link_libraries(${PROJECT_NAME}_extrinsic_hand_eye_tests PRIVATE ${PROJECT_NAME}_test_support GTest::GTest
                                                                       GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_extrinsic_hand_eye_tests)
add_dependencies(run_tests ${PROJECT_NAME}_extrinsic_hand_eye_tests)

# DH Chain
add_executable(${PROJECT_NAME}_dh_parameter_tests dh_parameter_utest.cpp)
target_link_libraries(${PROJECT_NAME}_dh_parameter_tests PRIVATE ${PROJECT_NAME}_test_support GTest::GTest GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_dh_parameter_tests)
add_dependencies(run_tests ${PROJECT_NAME}_dh_parameter_tests)

# Extrinsic hand-eye using DH Chain
add_executable(${PROJECT_NAME}_extrinsic_hand_eye_dh_chain_tests extrinsic_hand_eye_dh_chain_utest.cpp)
target_link_libraries(${PROJECT_NAME}_extrinsic_hand_eye_dh_chain_tests PRIVATE ${PROJECT_NAME}_test_support
                                                                                GTest::GTest GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_extrinsic_hand_eye_dh_chain_tests)
add_dependencies(run_tests ${PROJECT_NAME}_extrinsic_hand_eye_dh_chain_tests)

# Covariance The expectations for the covariance matrices of these tests tends to be flaky due to random number
# generation, so let's only build but not run these tests
add_executable(${PROJECT_NAME}_covariance_tests covariance_utest.cpp)
target_link_libraries(${PROJECT_NAME}_covariance_tests PRIVATE ${PROJECT_NAME}_test_support GTest::GTest GTest::Main)
add_dependencies(run_tests ${PROJECT_NAME}_covariance_tests)

# Covariance report
add_executable(${PROJECT_NAME}_covariance_report_tests covariance_report_utest.cpp)
target_link_libraries(${PROJECT_NAME}_covariance_report_tests PRIVATE ${PROJECT_NAME}_optimizations GTest::GTest
                                                                      GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_covariance_report_tests)
add_dependencies(run_tests ${PROJECT_NAME}_covariance_report_tests)

# Homography
add_executable(${PROJECT_NAME}_homography_tests homography_utest.cpp)
target_link_libraries(
  ${PROJECT_NAME}_homography_tests
  PRIVATE ${PROJECT_NAME}_test_support
          ${PROJECT_NAME}_analysis
          GTest::GTest
          GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_homography_tests)
add_dependencies(run_tests ${PROJECT_NAME}_homography_tests)

# PnP
add_executable(${PROJECT_NAME}_pnp_tests pnp_utest.cpp)
target_link_libraries(
  ${PROJECT_NAME}_pnp_tests
  PRIVATE ${PROJECT_NAME}_test_support
          ${PROJECT_NAME}_analysis
          GTest::GTest
          GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_pnp_tests)
add_dependencies(run_tests ${PROJECT_NAME}_pnp_tests)

# Camera intrinsic calibration validation
add_executable(${PROJECT_NAME}_camera_intrinsic_validation_tests camera_intrinsic_calibration_validation_utest.cpp)
target_link_libraries(
  ${PROJECT_NAME}_camera_intrinsic_validation_tests
  PRIVATE ${PROJECT_NAME}_test_support
          ${PROJECT_NAME}_analysis
          GTest::GTest
          GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_camera_intrinsic_validation_tests)
add_dependencies(run_tests ${PROJECT_NAME}_camera_intrinsic_validation_tests)

# Noise Qualification
add_executable(${PROJECT_NAME}_noise_tests noise_qualification_utest.cpp)
target_link_libraries(
  ${PROJECT_NAME}_noise_tests
  PRIVATE ${PROJECT_NAME}_test_support
          ${PROJECT_NAME}_analysis
          GTest::GTest
          GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_noise_tests)
add_dependencies(run_tests ${PROJECT_NAME}_noise_tests)

# Maximum Likelihood
add_executable(${PROJECT_NAME}_maximum_likelihood_tests maximum_likelihood_utest.cpp)
target_link_libraries(${PROJECT_NAME}_maximum_likelihood_tests PRIVATE ${PROJECT_NAME}_test_support GTest::GTest
                                                                       GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_maximum_likelihood_tests)
add_dependencies(run_tests ${PROJECT_NAME}_maximum_likelihood_tests)

# DH Chain Kinematic Calibration
add_executable(${PROJECT_NAME}_dh_chain_kinematic_calibration_tests dh_chain_kinematic_calibration_utest.cpp)
target_link_libraries(
  ${PROJECT_NAME}_dh_chain_kinematic_calibration_tests
  PRIVATE ${PROJECT_NAME}_test_support
          ${PROJECT_NAME}_analysis
          GTest::GTest
          GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_dh_chain_kinematic_calibration_tests)
add_dependencies(run_tests ${PROJECT_NAME}_dh_chain_kinematic_calibration_tests)

# DH Chain Kinematic Measurement Calibration
add_executable(${PROJECT_NAME}_dh_chain_kinematic_measurement_tests dh_chain_kinematic_measurement_utest.cpp)
target_link_libraries(
  ${PROJECT_NAME}_dh_chain_kinematic_measurement_tests
  PRIVATE ${PROJECT_NAME}_test_support
          ${PROJECT_NAME}_analysis
          GTest::GTest
          GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_dh_chain_kinematic_measurement_tests)
add_dependencies(run_tests ${PROJECT_NAME}_dh_chain_kinematic_measurement_tests)

# Local Parameterization
add_executable(${PROJECT_NAME}_local_parameterization_tests local_parameterization_utest.cpp)
target_link_libraries(${PROJECT_NAME}_local_parameterization_tests PRIVATE ${PROJECT_NAME}_test_support GTest::GTest
                                                                           GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_local_parameterization_tests)
add_dependencies(run_tests ${PROJECT_NAME}_local_parameterization_tests)

# Serialization
add_executable(${PROJECT_NAME}_serialization_tests serialization_utest.cpp)
target_link_libraries(${PROJECT_NAME}_serialization_tests PRIVATE ${PROJECT_NAME}_test_support GTest::GTest GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_serialization_tests)
add_dependencies(run_tests ${PROJECT_NAME}_serialization_tests)

# Target Finder
add_executable(${PROJECT_NAME}_target_finder_test target_finder_utest.cpp)
target_link_libraries(
  ${PROJECT_NAME}_target_finder_test
  PRIVATE ${PROJECT_NAME}_target_finders_opencv
          ${PROJECT_NAME}_analysis
          GTest::GTest
          GTest::Main)
add_gtest_discover_tests(${PROJECT_NAME}_target_finder_test)
add_dependencies(run_tests ${PROJECT_NAME}_target_finder_test)
target_compile_definitions(${PROJECT_NAME}_target_finder_test
                           PUBLIC TEST_SUPPORT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/images/")

# Install the test executables so they can be run independently later if needed
install(
  TARGETS ${PROJECT_NAME}_test_support
          ${PROJECT_NAME}_conversion_tests
          ${PROJECT_NAME}_covariance_tests
          ${PROJECT_NAME}_covariance_report_tests
          ${PROJECT_NAME}_extrinsic_multi_static_camera_tests
          ${PROJECT_NAME}_extrinsic_hand_eye_tests
          ${PROJECT_NAME}_dh_parameter_tests
          ${PROJECT_NAME}_extrinsic_hand_eye_dh_chain_tests
          ${PROJECT_NAME}_homography_tests
          ${PROJECT_NAME}_pnp_tests
          ${PROJECT_NAME}_camera_intrinsic_validation_tests
          ${PROJECT_NAME}_noise_tests
          ${PROJECT_NAME}_maximum_likelihood_tests
          ${PROJECT_NAME}_dh_chain_kinematic_calibration_tests
          ${PROJECT_NAME}_local_parameterization_tests
          ${PROJECT_NAME}_dh_chain_kinematic_measurement_tests
          ${PROJECT_NAME}_serialization_tests
          ${PROJECT_NAME}_target_finder_test
  RUNTIME DESTINATION bin/tests
  LIBRARY DESTINATION lib/tests
  ARCHIVE DESTINATION lib/tests)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND TARGET ${PROJECT_NAME}_camera_intrinsic_calibration_app
   AND TARGET ${PROJECT_NAME}_extrinsic_hand_eye_calibration_app)
  add_test(
    NAME HeadlessCalibration.OutputPathProtection
    COMMAND
      "${CMAKE_COMMAND}" "-DINTRINSIC_APP=$<TARGET_FILE:${PROJECT_NAME}_camera_intrinsic_calibration_app>"
      "-DHAND_EYE_APP=$<TARGET_FILE:${PROJECT_NAME}_extrinsic_hand_eye_calibration_app>"
      "-DPLUGIN_DIR=$<TARGET_FILE_DIR:${PROJECT_NAME}_plugins_opencv>"
      "-DSAMPLE_DIR=${PROJECT_SOURCE_DIR}/examples/data/test_set_10x10"
      "-DOUTPUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/headless_output_path_protection" -P
      "${CMAKE_CURRENT_SOURCE_DIR}/headless_output_path_protection.cmake")
  set_tests_properties(HeadlessCalibration.OutputPathProtection PROPERTIES TIMEOUT 30)
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND TARGET ${PROJECT_NAME}_camera_intrinsic_calibration_app
   AND TARGET ${PROJECT_NAME}_extrinsic_hand_eye_calibration_app)
  set(intrinsic_app ${PROJECT_NAME}_camera_intrinsic_calibration_app)
  set(hand_eye_app ${PROJECT_NAME}_extrinsic_hand_eye_calibration_app)
  set(plugin_target ${PROJECT_NAME}_plugins_opencv)
  add_dependencies(
    run_tests
    ${intrinsic_app}
    ${hand_eye_app}
    ${plugin_target})
  add_test(
    NAME HeadlessCalibration.ObservationWorkflow
    COMMAND
      "${CMAKE_COMMAND}" "-DINTRINSIC_APP=$<TARGET_FILE:${intrinsic_app}>"
      "-DHAND_EYE_APP=$<TARGET_FILE:${hand_eye_app}>" "-DPLUGIN_DIR=$<TARGET_FILE_DIR:${plugin_target}>"
      "-DSAMPLE_DIR=${PROJECT_SOURCE_DIR}/examples/data/test_set_10x10"
      "-DOUTPUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/headless_calibration_workflow" -P
      "${CMAKE_CURRENT_SOURCE_DIR}/headless_calibration_workflow.cmake")
  set_tests_properties(HeadlessCalibration.ObservationWorkflow PROPERTIES TIMEOUT 30)
endif()

if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND TARGET ${PROJECT_NAME}_camera_intrinsic_calibration_app)
  set(intrinsic_app ${PROJECT_NAME}_camera_intrinsic_calibration_app)
  set(resolution_fixture ${PROJECT_NAME}_intrinsic_resolution_fixture)
  set(plugin_target ${PROJECT_NAME}_plugins_opencv)
  add_executable(${resolution_fixture} intrinsic_resolution_fixture.cpp)
  target_link_libraries(${resolution_fixture} PRIVATE ${PROJECT_NAME}_target_finders_opencv)
  add_dependencies(
    run_tests
    ${intrinsic_app}
    ${resolution_fixture}
    ${plugin_target})
  add_test(
    NAME HeadlessCalibration.IntrinsicObservationResolution
    COMMAND
      "${CMAKE_COMMAND}" "-DINTRINSIC_APP=$<TARGET_FILE:${intrinsic_app}>"
      "-DRESOLUTION_FIXTURE=$<TARGET_FILE:${resolution_fixture}>" "-DPLUGIN_DIR=$<TARGET_FILE_DIR:${plugin_target}>"
      "-DSAMPLE_DIR=${PROJECT_SOURCE_DIR}/examples/data/test_set_10x10"
      "-DOUTPUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/intrinsic_observation_resolution" -P
      "${CMAKE_CURRENT_SOURCE_DIR}/intrinsic_observation_resolution.cmake")
  set_tests_properties(HeadlessCalibration.IntrinsicObservationResolution PROPERTIES TIMEOUT 30)
endif()
