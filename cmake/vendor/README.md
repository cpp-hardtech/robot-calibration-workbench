# Vendored build helpers

These CMake modules are third-party code, copied in verbatim so the build does not have to
fetch them at configure time. They define the helpers the top-level `CMakeLists.txt` uses:
`extract_package_metadata`, `configure_package`, `configure_component`, `install_targets`,
`generate_package_config`, `target_cxx_version`, `target_clang_tidy`, `find_gtest`,
`add_run_tests_target`, and `target_code_coverage`.

| File | Origin | License |
| --- | --- | --- |
| `extract_package_metadata.cmake` | ABB Schweiz AG | BSD 3-Clause |
| `cmake_tools.cmake` | George Cave, Southwest Research Institute | Apache License 2.0 |
| `code_coverage.cmake` | George Cave | Apache License 2.0 |

Taken at version 0.5.4.

## Local changes

The licensing headers in each file are retained unmodified. Two comment-only edits were
applied to `cmake_tools.cmake`:

- the header comment written into the generated package config text named the project the
  file came from; it now reads `Default *-config.cmake file`;
- a note referred to fetching this module with `find_package`; it now describes defining
  `CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS` before including the module.

No functional code was changed.

## Updating

To take a newer upstream version, copy the three files back in and reapply those two comment
edits. Then verify a clean configure still finds every helper listed above.
