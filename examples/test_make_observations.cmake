set(images "${TEST_DIRECTORY}/images")
set(poses "${TEST_DIRECTORY}/poses")
set(manifests "${TEST_DIRECTORY}/manifests")
file(
  MAKE_DIRECTORY
  "${images}/nested"
  "${poses}"
  "${manifests}"
  "${TEST_DIRECTORY}/empty")
file(WRITE "${images}/z.jpeg" "Image placeholder")
file(WRITE "${images}/A frame.PNG" "Image placeholder")
file(WRITE "${images}/ignore.txt" "")
file(WRITE "${images}/nested/ignore.png" "")
file(WRITE "${poses}/z.yaml" "Pose placeholder")
file(WRITE "${poses}/A frame.YML" "Pose placeholder")

execute_process(
  COMMAND "${MAKE_OBSERVATIONS}" "${images}"
  RESULT_VARIABLE status
  OUTPUT_VARIABLE manifest
  ERROR_VARIABLE diagnostic)
set(expected "data:\n  - image: ${images}/A frame.PNG\n  - image: ${images}/z.jpeg\n")
if(NOT
   status
   EQUAL
   0
   OR NOT
      manifest
      STREQUAL
      expected)
  message(FATAL_ERROR "Unexpected absolute observation manifest: ${diagnostic}\n${manifest}")
endif()

execute_process(
  COMMAND "${MAKE_OBSERVATIONS}" ./../images/. --poses ../poses --relative-to .
  WORKING_DIRECTORY "${manifests}"
  RESULT_VARIABLE status
  OUTPUT_VARIABLE manifest
  ERROR_VARIABLE diagnostic)
string(CONCAT expected "data:\n  - image: ../images/A frame.PNG\n    pose: ../poses/A frame.YML\n"
              "  - image: ../images/z.jpeg\n    pose: ../poses/z.yaml\n")
if(NOT
   status
   EQUAL
   0
   OR NOT
      manifest
      STREQUAL
      expected)
  message(FATAL_ERROR "Unexpected relative observation manifest: ${diagnostic}\n${manifest}")
endif()

function(expect_failure expected_status reason)
  execute_process(
    COMMAND "${MAKE_OBSERVATIONS}" ${ARGN}
    RESULT_VARIABLE status
    OUTPUT_VARIABLE manifest
    ERROR_VARIABLE diagnostic)
  if(NOT
     status
     EQUAL
     expected_status
     OR NOT
        manifest
        STREQUAL
        ""
     OR NOT
        diagnostic
        MATCHES
        "${reason}")
    message(FATAL_ERROR "Unexpected failure result (${status}): ${manifest}\n${diagnostic}")
  endif()
endfunction()

expect_failure(1 "No image files" "${TEST_DIRECTORY}/empty")
expect_failure(
  1
  "Expected an existing manifest directory"
  "${images}"
  --relative-to
  "${manifests}/missing")
expect_failure(
  2
  "Usage:"
  "${images}"
  --poses)
expect_failure(
  2
  "Usage:"
  "${images}"
  --relative-to
  "${manifests}"
  --relative-to
  "${manifests}")
set(duplicate_images "${TEST_DIRECTORY}/duplicate-images")
set(duplicate_poses "${TEST_DIRECTORY}/duplicate-poses")
set(unmatched_poses "${TEST_DIRECTORY}/unmatched-poses")
file(
  MAKE_DIRECTORY
  "${duplicate_images}"
  "${duplicate_poses}"
  "${unmatched_poses}")

file(WRITE "${duplicate_images}/A frame.PNG" "")
file(WRITE "${duplicate_images}/A frame.jpg" "")
expect_failure(1 "Duplicate image stem" "${duplicate_images}")

file(WRITE "${duplicate_poses}/A frame.YML" "")
file(WRITE "${duplicate_poses}/A frame.yaml" "")
expect_failure(
  1
  "Duplicate pose stem"
  "${images}"
  --poses
  "${duplicate_poses}")

file(WRITE "${unmatched_poses}/A frame.YML" "")
file(WRITE "${unmatched_poses}/Z.yaml" "")
expect_failure(
  1
  "Missing pose for image"
  "${images}"
  --poses
  "${unmatched_poses}")

set(dataset "${CMAKE_CURRENT_LIST_DIR}/data/test_set_10x10")
set(session_images "${TEST_DIRECTORY}/session-images")
set(session_poses "${TEST_DIRECTORY}/session-poses")
foreach(session IN ITEMS first second)
  file(MAKE_DIRECTORY "${session_images}/${session}" "${session_poses}/${session}")
endforeach()
configure_file("${dataset}/images/0.png" "${session_images}/first/frame.png" COPYONLY)
configure_file("${dataset}/images/1.png" "${session_images}/second/frame.png" COPYONLY)
configure_file("${dataset}/poses/0.yaml" "${session_poses}/first/frame.yaml" COPYONLY)
configure_file("${dataset}/poses/1.yaml" "${session_poses}/second/frame.yml" COPYONLY)

execute_process(
  COMMAND "${MAKE_OBSERVATIONS}" ../session-images --relative-to . --recursive --poses ../session-poses
  WORKING_DIRECTORY "${manifests}"
  RESULT_VARIABLE status
  OUTPUT_VARIABLE manifest
  ERROR_VARIABLE diagnostic)
string(CONCAT expected
              "data:\n  - image: ../session-images/first/frame.png\n    pose: ../session-poses/first/frame.yaml\n"
              "  - image: ../session-images/second/frame.png\n    pose: ../session-poses/second/frame.yml\n")
if(NOT
   status
   EQUAL
   0
   OR NOT
      manifest
      STREQUAL
      expected)
  message(FATAL_ERROR "Unexpected recursive observation manifest (${status}): ${diagnostic}\n${manifest}")
endif()

expect_failure(1 "No image files" "${session_images}")
expect_failure(
  2
  "Usage:"
  "${session_images}"
  --recursive
  --recursive)

configure_file("${dataset}/poses/0.yaml" "${session_poses}/first/frame.yml" COPYONLY)
expect_failure(
  1
  "Duplicate pose stem 'first/frame'"
  "${session_images}"
  --recursive
  --poses
  "${session_poses}")
file(REMOVE "${session_poses}/first/frame.yml" "${session_poses}/second/frame.yml")
expect_failure(
  1
  "Missing pose for image .*/second/frame.png"
  "${session_images}"
  --poses
  "${session_poses}"
  --recursive)
