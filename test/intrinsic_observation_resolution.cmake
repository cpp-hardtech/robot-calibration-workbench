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

file(MAKE_DIRECTORY "${OUTPUT_DIR}")
set(config "${SAMPLE_DIR}/cal_data.yaml")
set(padded "${OUTPUT_DIR}/padded.png")
set(unreadable "${OUTPUT_DIR}/unreadable.png")
execute_process(COMMAND "${RESOLUTION_FIXTURE}" "${SAMPLE_DIR}/images/0.png" "${padded}" RESULT_VARIABLE fixture_status)
if(NOT
   fixture_status
   STREQUAL
   "0")
  message(FATAL_ERROR "Could not pad the real sample image")
endif()
file(WRITE "${unreadable}" "not a PNG")

file(READ "${config}" config_yaml)
string(
  REPLACE "use_opencv: false"
          "use_opencv: true"
          opencv_yaml
          "${config_yaml}")
set(opencv_config "${OUTPUT_DIR}/opencv.yaml")
file(WRITE "${opencv_config}" "${opencv_yaml}")

file(GLOB images "${SAMPLE_DIR}/images/*.png")
set(ordinary_rows "")
set(remaining_rows "")
foreach(image IN LISTS images)
  get_filename_component(stem "${image}" NAME_WE)
  set(row "  - image: \"${image}\"\n    pose: \"${SAMPLE_DIR}/poses/${stem}.yaml\"\n")
  string(APPEND ordinary_rows "${row}")
  if(NOT
     stem
     STREQUAL
     "0")
    string(APPEND remaining_rows "${row}")
  endif()
endforeach()

set(padded_row "  - image: \"${padded}\"\n    pose: \"${SAMPLE_DIR}/poses/0.yaml\"\n")
set(unreadable_row "  - image: \"${unreadable}\"\n    pose: \"${SAMPLE_DIR}/poses/0.yaml\"\n")
file(WRITE "${OUTPUT_DIR}/padded-first.yaml" "data:\n${padded_row}${remaining_rows}")
file(WRITE "${OUTPUT_DIR}/padded-last.yaml" "data:\n${remaining_rows}${padded_row}")
file(WRITE "${OUTPUT_DIR}/unreadable-first.yaml" "data:\n${unreadable_row}${ordinary_rows}")

function(
  run_calibration
  mode
  cfg
  expected
  reason)
  set(result "${OUTPUT_DIR}/${mode}-result.yaml")
  set(ros "${OUTPUT_DIR}/${mode}-ros.yaml")
  set(report "${OUTPUT_DIR}/${mode}-report.csv")
  file(
    REMOVE
    "${result}"
    "${ros}"
    "${report}")
  execute_process(COMMAND "${INTRINSIC_APP}" --headless "${cfg}" "${OUTPUT_DIR}/${mode}.yaml" "${result}" "${ros}"
                          --observation-report "${report}" RESULT_VARIABLE status ERROR_VARIABLE error)
  if(NOT
     status
     STREQUAL
     "${expected}"
     OR NOT
        error
        MATCHES
        "${reason}")
    message(FATAL_ERROR "${mode} returned ${status}, expected ${expected}: ${error}")
  endif()
  if(expected STREQUAL "0")
    if(NOT EXISTS "${result}")
      message(FATAL_ERROR "${mode} did not write calibration results")
    endif()
    file(READ "${ros}" ros_yaml)
    file(STRINGS "${report}" included_rows REGEX "Included in calibration")
    list(LENGTH included_rows included_count)
    file(READ "${report}" report_csv)
    if(NOT
       ros_yaml
       MATCHES
       "image_width: 640"
       OR NOT
          ros_yaml
          MATCHES
          "image_height: 480"
       OR NOT
          included_count
          EQUAL
          15
       OR NOT
          report_csv
          MATCHES
          "unreadable.png.*Failed to load file")
      message(FATAL_ERROR "${mode} did not preserve the image-read diagnostic and 15 usable 640x480 observations")
    endif()
    message(STATUS "${mode}: 15 usable images, unreadable image reported, ROS 640x480")
  else()
    if(EXISTS "${result}" OR EXISTS "${ros}" OR EXISTS "${report}")
      message(FATAL_ERROR "${mode} wrote outputs after rejecting the mixed-size dataset")
    endif()
    message(STATUS "${mode}: ${error}")
  endif()
endfunction()

run_calibration(
  padded-first
  "${config}"
  1
  "Mixed image resolutions: image '.*images/1.png' is 640x480 \\(expected 704x480\\)")
run_calibration(
  padded-last
  "${opencv_config}"
  1
  "Mixed image resolutions: image '.*padded.png' is 704x480 \\(expected 640x480\\)")
run_calibration(
  unreadable-first
  "${config}"
  0
  "")
