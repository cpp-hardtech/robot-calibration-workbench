set(library_path "${PLUGIN_DIR}")
if(NOT
   "$ENV{LD_LIBRARY_PATH}"
   STREQUAL
   "")
  string(APPEND library_path ":$ENV{LD_LIBRARY_PATH}")
endif()
set(ENV{LD_LIBRARY_PATH} "${library_path}")
unset(ENV{DISPLAY})
unset(ENV{WAYLAND_DISPLAY})

set(observations "${SAMPLE_DIR}/cal_data.yaml")
file(
  MAKE_DIRECTORY
  "${OUTPUT_DIR}/config"
  "${OUTPUT_DIR}/calibration"
  "${OUTPUT_DIR}/run")

function(
  run_headless
  executable
  config
  result
  expected
  reason)
  file(REMOVE "${result}")
  execute_process(
    COMMAND "${executable}" --headless "${config}" "${observations}" "${result}" ${ARGN}
    WORKING_DIRECTORY "${OUTPUT_DIR}/run"
    RESULT_VARIABLE status
    ERROR_VARIABLE error)
  if(NOT
     status
     STREQUAL
     "${expected}"
     OR NOT
        error
        MATCHES
        "${reason}")
    message(FATAL_ERROR "${executable} returned ${status}, expected ${expected}: ${error}")
  endif()
  if((expected STREQUAL "0" AND NOT EXISTS "${result}")
     OR (NOT
         expected
         STREQUAL
         "0"
         AND EXISTS "${result}"
        ))
    message(FATAL_ERROR "Unexpected result-file state for ${result}")
  endif()
endfunction()

set(ros_file "${OUTPUT_DIR}/calibration/camera.yaml")
file(REMOVE "${ros_file}")
run_headless(
  "${INTRINSIC_APP}"
  "${observations}"
  "${OUTPUT_DIR}/source-result.yaml"
  0
  ""
  "${ros_file}")
file(READ "${ros_file}" ros_yaml)
string(
  REPLACE "plumb_bob"
          "equidistant"
          unsupported_yaml
          "${ros_yaml}")
file(WRITE "${OUTPUT_DIR}/calibration/unsupported-camera.yaml" "${unsupported_yaml}")

file(READ "${observations}" config)
set(reference "  ros_camera_info: ../calibration/camera.yaml\n")
file(WRITE "${OUTPUT_DIR}/config/intrinsic.yaml" "${config}\nintrinsics_guess:\n${reference}")

set(inline_intrinsics "\nintrinsics:\n( +[^\n]*\n)+")
string(
  REGEX
  REPLACE "${inline_intrinsics}"
          "\nintrinsics:\n${reference}"
          hand_eye
          "${config}")
if(hand_eye STREQUAL config)
  message(FATAL_ERROR "Could not replace the sample's inline intrinsics")
endif()
file(WRITE "${OUTPUT_DIR}/config/hand-eye.yaml" "${hand_eye}")

run_headless(
  "${INTRINSIC_APP}"
  "${OUTPUT_DIR}/config/intrinsic.yaml"
  "${OUTPUT_DIR}/intrinsic-result.yaml"
  0
  "")
run_headless(
  "${HAND_EYE_APP}"
  "${OUTPUT_DIR}/config/hand-eye.yaml"
  "${OUTPUT_DIR}/hand-eye-result.yaml"
  0
  "")

string(
  REPLACE "calibration/camera.yaml"
          "calibration/unsupported-camera.yaml"
          unsupported_config
          "${hand_eye}")
file(WRITE "${OUTPUT_DIR}/config/unsupported.yaml" "${unsupported_config}")
run_headless(
  "${HAND_EYE_APP}"
  "${OUTPUT_DIR}/config/unsupported.yaml"
  "${OUTPUT_DIR}/unsupported-result.yaml"
  1
  "ROS camera calibration.*unsupported-camera.yaml.*plumb_bob")
