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

set(quoted_image "${TEST_DIRECTORY}/frame \"left\", 01.jpg")
set(blank "${TEST_DIRECTORY}/blank frame.pgm")
set(unreadable "${TEST_DIRECTORY}/missing frame.jpg")
configure_file("${TARGET_IMAGE}" "${quoted_image}" COPYONLY)
set(white_pixels "")
foreach(i RANGE 1 256)
  string(APPEND white_pixels "255 255 255 255\n")
endforeach()
file(WRITE "${blank}" "P2\n32 32\n255\n${white_pixels}")
string(
  REPLACE "\""
          "\"\""
          escaped_image
          "${quoted_image}")
set(batch_header "image_path,status,feature_count,point_count\n")
set(detected_row "\"${escaped_image}\",detected,24,24\n")
string(
  CONCAT expected_batch
         "${batch_header}"
         "\"${blank}\",no_detection,0,0\n"
         "\"${unreadable}\",unreadable,0,0\n"
         "${detected_row}")

execute_process(
  COMMAND "${DETECT_TARGET}" "${TARGET_CONFIG}" --batch "${blank}" "${unreadable}" "${quoted_image}"
  RESULT_VARIABLE status
  OUTPUT_VARIABLE batch
  ERROR_VARIABLE diagnostic)
if(NOT
   status
   EQUAL
   1
   OR NOT
      batch
      STREQUAL
      expected_batch)
  message(FATAL_ERROR "Unexpected mixed batch result: ${status}\n${batch}\n${diagnostic}")
endif()

execute_process(
  COMMAND "${DETECT_TARGET}" "${TARGET_CONFIG}" --batch "${quoted_image}" "${quoted_image}"
  RESULT_VARIABLE status
  OUTPUT_VARIABLE batch
  ERROR_VARIABLE diagnostic)
set(expected_batch "${batch_header}${detected_row}${detected_row}")
if(NOT
   status
   EQUAL
   0
   OR NOT
      batch
      STREQUAL
      expected_batch)
  message(FATAL_ERROR "Unexpected successful batch result: ${status}\n${batch}\n${diagnostic}")
endif()

execute_process(
  COMMAND "${DETECT_TARGET}" "${TARGET_CONFIG}" --batch
  RESULT_VARIABLE status
  OUTPUT_VARIABLE batch
  ERROR_VARIABLE diagnostic)
if(NOT
   status
   EQUAL
   2
   OR NOT
      batch
      STREQUAL
      ""
   OR NOT
      diagnostic
      MATCHES
      "Usage:")
  message(FATAL_ERROR "An empty batch did not produce a usage error")
endif()

foreach(
  option IN
  ITEMS --overlay
        --correspondences
        --batch
        --unknown)
  execute_process(
    COMMAND "${DETECT_TARGET}" "${TARGET_CONFIG}" --batch "${quoted_image}" "${option}"
    RESULT_VARIABLE status
    OUTPUT_VARIABLE batch
    ERROR_VARIABLE diagnostic)
  if(NOT
     status
     EQUAL
     2
     OR NOT
        batch
        STREQUAL
        ""
     OR NOT
        diagnostic
        MATCHES
        "Usage:"
     OR NOT
        diagnostic
        MATCHES
        "${option}")
    message(
      FATAL_ERROR
        "Batch option ${option}: expected usage status 2 and empty CSV, got ${status}\n${batch}\n${diagnostic}")
  endif()
endforeach()

configure_file("${TARGET_IMAGE}" "${TEST_DIRECTORY}/--camera.jpg" COPYONLY)
execute_process(
  COMMAND "${DETECT_TARGET}" "${TARGET_CONFIG}" --batch "./--camera.jpg"
  WORKING_DIRECTORY "${TEST_DIRECTORY}"
  RESULT_VARIABLE status
  OUTPUT_VARIABLE batch
  ERROR_VARIABLE diagnostic)
if(NOT
   status
   EQUAL
   0
   OR NOT
      batch
      MATCHES
      "^image_path,status,feature_count,point_count\n\"[.]/--camera[.]jpg\",detected,")
  message(FATAL_ERROR "An explicit path to a dash-prefixed image failed (${status})\n${batch}\n${diagnostic}")
endif()
