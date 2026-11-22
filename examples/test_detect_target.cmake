set(library_path "${PLUGIN_DIRECTORY}")
if(NOT
   "$ENV{LD_LIBRARY_PATH}"
   STREQUAL
   "")
  string(APPEND library_path ":$ENV{LD_LIBRARY_PATH}")
endif()
set(ENV{LD_LIBRARY_PATH} "${library_path}")

file(MAKE_DIRECTORY "${TEST_DIRECTORY}")
set(overlay "${TEST_DIRECTORY}/overlay.png")
file(REMOVE "${overlay}")

execute_process(
  COMMAND "${DETECT_TARGET}" "${TARGET_CONFIG}" "${TARGET_IMAGE}"
  RESULT_VARIABLE status
  OUTPUT_VARIABLE detections
  ERROR_VARIABLE diagnostic)
if(NOT
   status
   EQUAL
   0
   OR NOT
      detections
      MATCHES
      "^feature_id,point_index,x,y\n")
  message(FATAL_ERROR "Unexpected default CSV: ${diagnostic}\n${detections}")
endif()

execute_process(
  COMMAND "${DETECT_TARGET}" "${TARGET_CONFIG}" "${TARGET_IMAGE}" --overlay "${overlay}"
  RESULT_VARIABLE status
  OUTPUT_VARIABLE annotated_detections
  ERROR_VARIABLE diagnostic)
if(NOT
   status
   EQUAL
   0
   OR NOT
      annotated_detections
      STREQUAL
      detections
   OR NOT EXISTS "${overlay}")
  message(FATAL_ERROR "Failed to save an overlay with the default CSV: ${diagnostic}")
endif()
file(READ "${overlay}" signature LIMIT 8 HEX)
if(NOT
   signature
   STREQUAL
   "89504e470d0a1a0a")
  message(FATAL_ERROR "Overlay is not a PNG")
endif()

execute_process(
  COMMAND "${DETECT_TARGET}" "${TARGET_CONFIG}" "${TARGET_IMAGE}" --correspondences
  RESULT_VARIABLE status
  OUTPUT_VARIABLE correspondences
  ERROR_VARIABLE diagnostic)
string(
  REGEX MATCHALL
        "\n"
        rows
        "${correspondences}")
list(LENGTH rows row_count)
if(NOT
   status
   EQUAL
   0
   OR NOT
      correspondences
      MATCHES
      "^image_x,image_y,target_x,target_y,target_z\n"
   OR NOT
      row_count
      EQUAL
      25)
  message(FATAL_ERROR "Unexpected correspondence CSV: ${diagnostic}\n${correspondences}")
endif()

execute_process(
  COMMAND "${DETECT_TARGET}" "${TARGET_CONFIG}" "${TARGET_IMAGE}" --overlay
  RESULT_VARIABLE status
  OUTPUT_VARIABLE output
  ERROR_VARIABLE diagnostic)
if(NOT
   status
   EQUAL
   2
   OR NOT
      output
      STREQUAL
      ""
   OR NOT
      diagnostic
      MATCHES
      "--overlay")
  message(FATAL_ERROR "A missing overlay path did not produce a usage error")
endif()

set(missing_directory "${TEST_DIRECTORY}/missing")
execute_process(
  COMMAND "${DETECT_TARGET}" "${TARGET_CONFIG}" "${TARGET_IMAGE}" --overlay "${missing_directory}/overlay.png"
  RESULT_VARIABLE status
  OUTPUT_VARIABLE output
  ERROR_VARIABLE diagnostic)
if(NOT
   status
   EQUAL
   1
   OR NOT
      output
      STREQUAL
      ""
   OR NOT
      diagnostic
      MATCHES
      "Failed to write target overlay")
  message(FATAL_ERROR "An unwritable overlay path did not produce an output error: ${diagnostic}")
endif()
