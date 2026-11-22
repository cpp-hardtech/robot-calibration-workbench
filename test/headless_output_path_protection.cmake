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

file(MAKE_DIRECTORY "${OUTPUT_DIR}/links")
set(config "${OUTPUT_DIR}/config.yaml")
set(observations "${OUTPUT_DIR}/observations.yaml")
set(unwritten "${OUTPUT_DIR}/unused-result.yaml" "${OUTPUT_DIR}/combined.yaml" "${OUTPUT_DIR}/combined-dangling.yaml")
file(REMOVE ${unwritten} "${OUTPUT_DIR}/config-hardlink.yaml")

configure_file("${SAMPLE_DIR}/cal_data.yaml" "${config}" COPYONLY)
configure_file("${SAMPLE_DIR}/cal_data.yaml" "${observations}" COPYONLY)

function(make_symlink target link)
  file(REMOVE "${OUTPUT_DIR}/${link}")
  execute_process(COMMAND "${CMAKE_COMMAND}" -E create_symlink "${target}" "${link}" WORKING_DIRECTORY "${OUTPUT_DIR}"
                  RESULT_VARIABLE status)
  if(NOT
     status
     STREQUAL
     "0")
    message(FATAL_ERROR "Could not create fixture symlink ${link}")
  endif()
endfunction()
make_symlink("${SAMPLE_DIR}/images" images)
make_symlink("${SAMPLE_DIR}/poses" poses)
make_symlink("observations.yaml" observations-link.yaml)
make_symlink("../combined-dangling.yaml" links/native.yaml)
make_symlink("loop-b.yaml" loop-a.yaml)
make_symlink("loop-a.yaml" loop-b.yaml)

execute_process(COMMAND ln "${config}" "${OUTPUT_DIR}/config-hardlink.yaml" RESULT_VARIABLE hardlink_status)
if(NOT
   hardlink_status
   STREQUAL
   "0")
  message(FATAL_ERROR "Could not create fixture hard link")
endif()
file(SHA256 "${config}" config_before)
file(SHA256 "${observations}" observations_before)

function(
  run_headless
  app
  expected
  reason)
  execute_process(
    COMMAND "${app}" --headless ${ARGN}
    WORKING_DIRECTORY "${OUTPUT_DIR}"
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
    message(FATAL_ERROR "${ARGN}: returned ${status}, expected ${expected}: ${error}")
  endif()
  if(NOT
     expected
     STREQUAL
     "0")
    message(STATUS "${error}")
  endif()
endfunction()

run_headless(
  "${INTRINSIC_APP}"
  1
  "calibration result path.*[.]/config.yaml.*configuration path.*config.yaml"
  config.yaml
  observations.yaml
  ./config.yaml)
run_headless(
  "${INTRINSIC_APP}"
  1
  "ROS calibration result path.*observations-link.yaml.*observation manifest path.*observations.yaml"
  config.yaml
  observations.yaml
  unused-result.yaml
  observations-link.yaml)
run_headless(
  "${HAND_EYE_APP}"
  1
  "calibration result path.*config-hardlink.yaml.*configuration path.*config.yaml"
  config.yaml
  observations.yaml
  config-hardlink.yaml)
run_headless(
  "${HAND_EYE_APP}"
  1
  "calibration result path.*observations.yaml.*observation manifest path.*observations.yaml"
  config.yaml
  observations.yaml
  observations.yaml)
run_headless(
  "${INTRINSIC_APP}"
  1
  "ROS calibration result path.*combined.yaml.*calibration result path.*combined.yaml"
  config.yaml
  observations.yaml
  combined.yaml
  ./combined.yaml)
run_headless(
  "${INTRINSIC_APP}"
  1
  "ROS calibration result path.*combined-dangling.yaml.*calibration result path.*links/native.yaml"
  config.yaml
  observations.yaml
  links/native.yaml
  combined-dangling.yaml)
run_headless(
  "${INTRINSIC_APP}"
  1
  "Too many symbolic links in calibration result path.*loop-a.yaml"
  config.yaml
  observations.yaml
  loop-a.yaml)
run_headless(
  "${HAND_EYE_APP}"
  1
  "Observation report path must differ"
  config.yaml
  observations.yaml
  unused-result.yaml
  --observation-report
  config.yaml)

foreach(path IN LISTS unwritten)
  if(EXISTS "${path}")
    message(FATAL_ERROR "Path protection allowed a write to ${path}")
  endif()
endforeach()
file(SHA256 "${config}" config_after)
file(SHA256 "${observations}" observations_after)
file(SHA256 "${OUTPUT_DIR}/config-hardlink.yaml" hardlink_after)
if(NOT
   config_before
   STREQUAL
   config_after
   OR NOT
      observations_before
      STREQUAL
      observations_after
   OR NOT
      config_before
      STREQUAL
      hardlink_after)
  message(FATAL_ERROR "Path protection modified the configuration, manifest, or hard link")
endif()

file(WRITE "${OUTPUT_DIR}/intrinsic-result.yaml" "old intrinsic results\n")
file(WRITE "${OUTPUT_DIR}/hand-eye-result.yaml" "old hand-eye results\n")
file(WRITE "${OUTPUT_DIR}/ros-result.yaml" "old ROS results\n")
run_headless(
  "${INTRINSIC_APP}"
  0
  ""
  config.yaml
  config.yaml
  intrinsic-result.yaml
  ros-result.yaml)
run_headless(
  "${HAND_EYE_APP}"
  0
  ""
  config.yaml
  config.yaml
  hand-eye-result.yaml)
file(READ "${OUTPUT_DIR}/intrinsic-result.yaml" intrinsic_output)
file(READ "${OUTPUT_DIR}/hand-eye-result.yaml" hand_eye_output)
file(READ "${OUTPUT_DIR}/ros-result.yaml" ros_output)
if(NOT
   intrinsic_output
   MATCHES
   "converged: true"
   OR NOT
      hand_eye_output
      MATCHES
      "converged: true"
   OR NOT
      ros_output
      MATCHES
      "camera_matrix:")
  message(FATAL_ERROR "Valid distinct outputs could not be overwritten")
endif()
message(STATUS "Identical inputs and distinct existing result paths succeeded in both applications")
