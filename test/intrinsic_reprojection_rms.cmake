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
set(manifest "${SAMPLE_DIR}/cal_data.yaml")
file(READ "${manifest}" source_config)
string(
  REPLACE "use_opencv: false"
          "use_opencv: true"
          opencv_config
          "${source_config}")
set(opencv_config_file "${OUTPUT_DIR}/opencv-config.yaml")
file(WRITE "${opencv_config_file}" "${opencv_config}")

function(run_calibration solver config)
  set(result "${OUTPUT_DIR}/${solver}-result.yaml")
  set(report "${OUTPUT_DIR}/${solver}-report.csv")
  file(REMOVE "${result}" "${report}")
  execute_process(COMMAND "${INTRINSIC_APP}" --headless "${config}" "${manifest}" "${result}" --observation-report
                          "${report}" RESULT_VARIABLE status ERROR_VARIABLE error)
  if(NOT
     status
     STREQUAL
     "0")
    message(FATAL_ERROR "${solver} calibration failed: ${error}")
  endif()
  file(STRINGS "${report}" accepted REGEX "100,[0-9.]+,\"Included in calibration\"")
  list(LENGTH accepted accepted_count)
  if(NOT
     accepted_count
     EQUAL
     15)
    message(FATAL_ERROR "${solver} did not calibrate with the expected 15 observations")
  endif()
endfunction()

run_calibration(ceres "${manifest}")
run_calibration(opencv "${opencv_config_file}")
execute_process(COMMAND "${RMS_CHECK}" "${OUTPUT_DIR}/ceres-result.yaml" "${OUTPUT_DIR}/opencv-result.yaml"
                RESULT_VARIABLE status)
if(NOT
   status
   STREQUAL
   "0")
  message(FATAL_ERROR "Intrinsic calibration did not report consistent reprojection RMS")
endif()
