set(library_path "${PLUGIN_DIRECTORY}")
if(NOT
   "$ENV{LD_LIBRARY_PATH}"
   STREQUAL
   "")
  string(APPEND library_path ":$ENV{LD_LIBRARY_PATH}")
endif()
set(ENV{LD_LIBRARY_PATH} "${library_path}")
unset(ENV{DISPLAY})
unset(ENV{WAYLAND_DISPLAY})

file(MAKE_DIRECTORY "${TEST_DIRECTORY}")
set(target "${SAMPLE_DIRECTORY}/cal_data.yaml")
set(camera "${SAMPLE_DIRECTORY}/camera.ros.yaml")
set(image "${SAMPLE_DIRECTORY}/images/0.png")
execute_process(
  COMMAND "${ESTIMATE_POSE}" "${target}" "${camera}" "${image}"
  RESULT_VARIABLE status
  OUTPUT_VARIABLE pose
  ERROR_VARIABLE diagnostic)
if(NOT
   status
   EQUAL
   0
   OR NOT
      pose
      MATCHES
      "^target_to_camera:\n")
  message(FATAL_ERROR "Failed to estimate the ChArUco target pose: ${diagnostic}")
endif()

foreach(
  field IN
  ITEMS x
        y
        z
        qw
        qx
        qy
        qz)
  if(NOT
     pose
     MATCHES
     "\n  ${field}: [+0-9.eE-]+")
    message(FATAL_ERROR "Missing transform field ${field}: ${pose}")
  endif()
endforeach()
string(
  REGEX MATCH
        "correspondence_count: ([0-9]+)"
        count
        "${pose}")
if(NOT count OR CMAKE_MATCH_1 LESS 4)
  message(FATAL_ERROR "Expected at least four correspondences: ${pose}")
endif()
string(
  REGEX MATCH
        "reprojection_rms_px: ([+0-9.eE-]+)"
        rms
        "${pose}")
if(NOT rms
   OR NOT
      CMAKE_MATCH_1
      LESS
      0.5)
  message(FATAL_ERROR "Expected a reprojection RMS under 0.5 px: ${pose}")
endif()

function(
  expect_failure
  camera_config
  input
  expected)
  execute_process(
    COMMAND "${ESTIMATE_POSE}" "${target}" "${camera_config}" "${input}"
    RESULT_VARIABLE status
    OUTPUT_VARIABLE pose
    ERROR_VARIABLE diagnostic)
  if(NOT
     status
     EQUAL
     1
     OR NOT
        pose
        STREQUAL
        ""
     OR NOT
        diagnostic
        MATCHES
        "${expected}")
    message(FATAL_ERROR "Unexpected pose failure result (${status}): ${pose}\n${diagnostic}")
  endif()
endfunction()

expect_failure("${camera}" "${TEST_DIRECTORY}/missing.png" "Failed to load file")
expect_failure("${camera}" "${FEW_CORNERS_IMAGE}" "Image dimensions must match")

file(READ "${camera}" camera_yaml)
string(
  REPLACE "plumb_bob"
          "equidistant"
          invalid_model
          "${camera_yaml}")
set(invalid_camera "${TEST_DIRECTORY}/invalid-camera.yaml")
file(WRITE "${invalid_camera}" "${invalid_model}")
expect_failure("${invalid_camera}" "${image}" "requires the plumb_bob distortion model")

string(
  REPLACE "image_width: 1600"
          "image_width: 960"
          small_camera
          "${camera_yaml}")
string(
  REPLACE "image_height: 1200"
          "image_height: 540"
          small_camera
          "${small_camera}")
set(small_camera_yaml "${TEST_DIRECTORY}/small-camera.yaml")
file(WRITE "${small_camera_yaml}" "${small_camera}")
expect_failure("${small_camera_yaml}" "${FEW_CORNERS_IMAGE}" "At least four target correspondences")
