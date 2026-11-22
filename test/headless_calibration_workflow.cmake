file(MAKE_DIRECTORY "${OUTPUT_DIR}")
set(config "${SAMPLE_DIR}/cal_data.yaml")
set(image_only "${OUTPUT_DIR}/image-only.yaml")
set(missing_pose_files "${OUTPUT_DIR}/missing-pose-files.yaml")
set(intrinsic_result "${OUTPUT_DIR}/intrinsic.yaml")
set(ros_result "${OUTPUT_DIR}/ros.yaml")
set(intrinsic_report "${OUTPUT_DIR}/intrinsic.csv")
set(hand_eye_result "${OUTPUT_DIR}/hand-eye.yaml")
set(hand_eye_report "${OUTPUT_DIR}/hand-eye.csv")
set(rejected_result "${OUTPUT_DIR}/rejected.yaml")
file(
  REMOVE
  "${intrinsic_result}"
  "${ros_result}"
  "${intrinsic_report}"
  "${hand_eye_result}"
  "${hand_eye_report}"
  "${rejected_result}")

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

file(GLOB images "${SAMPLE_DIR}/images/*.png")
file(WRITE "${image_only}" "data:\n  - image: missing-image.png\n")
file(WRITE "${missing_pose_files}" "data:\n")
foreach(image IN LISTS images)
  file(APPEND "${image_only}" "  - image: \"${image}\"\n")
  file(APPEND "${missing_pose_files}" "  - image: \"${image}\"\n    pose: missing-pose.yaml\n")
endforeach()
file(READ "${config}" config_yaml)
string(
  REPLACE "use_opencv: false"
          "use_opencv: true"
          opencv_yaml
          "${config_yaml}")
string(
  REPLACE "use_extrinsic_guesses: false"
          "use_extrinsic_guesses: true"
          guesses_yaml
          "${config_yaml}")
file(WRITE "${OUTPUT_DIR}/opencv.yaml" "${opencv_yaml}")
file(WRITE "${OUTPUT_DIR}/with-guesses.yaml" "${guesses_yaml}")

function(
  run_calibration
  executable
  expected
  reason)
  execute_process(COMMAND "${executable}" --headless ${ARGN} RESULT_VARIABLE status ERROR_VARIABLE error)
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
endfunction()

run_calibration(
  "${HAND_EYE_APP}"
  1
  "No usable observations"
  "${config}"
  "${missing_pose_files}"
  "${rejected_result}")

run_calibration(
  "${INTRINSIC_APP}"
  0
  ""
  "${OUTPUT_DIR}/opencv.yaml"
  "${image_only}"
  "${intrinsic_result}"
  "${ros_result}"
  --observation-report
  "${intrinsic_report}")
run_calibration(
  "${HAND_EYE_APP}"
  0
  ""
  "${config}"
  "${config}"
  "${hand_eye_result}"
  --observation-report
  "${hand_eye_report}")
run_calibration(
  "${INTRINSIC_APP}"
  1
  "pose"
  "${OUTPUT_DIR}/with-guesses.yaml"
  "${image_only}"
  "${rejected_result}")
run_calibration(
  "${HAND_EYE_APP}"
  1
  "pose"
  "${config}"
  "${image_only}"
  "${rejected_result}")
run_calibration(
  "${HAND_EYE_APP}"
  1
  "must differ"
  "${config}"
  "${config}"
  "${rejected_result}"
  --observation-report
  "${rejected_result}")

if(NOT EXISTS "${intrinsic_result}" OR NOT EXISTS "${hand_eye_result}" OR EXISTS "${rejected_result}")
  message(FATAL_ERROR "Unexpected calibration result files")
endif()
file(READ "${ros_result}" ros_yaml)
file(READ "${intrinsic_report}" intrinsic_csv)
file(READ "${hand_eye_report}" hand_eye_csv)
if(NOT
   ros_yaml
   MATCHES
   "image_width: 640"
   OR NOT
      ros_yaml
      MATCHES
      "image_height: 480")
  message(FATAL_ERROR "ROS calibration did not retain the accepted image size: ${ros_yaml}")
endif()
if(NOT
   intrinsic_csv
   MATCHES
   "\"\",100,[0-9.]+,\"Included in calibration\""
   OR NOT
      intrinsic_csv
      MATCHES
      "missing-image.png\",\"\",,,\"Image file does not exist\""
   OR NOT
      hand_eye_csv
      MATCHES
      "poses/0.yaml\",100,[0-9.]+,\"Included in calibration\"")
  message(FATAL_ERROR "Observation report does not contain the expected diagnostics")
endif()
