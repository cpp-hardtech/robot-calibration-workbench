#!/usr/bin/env bash
set -euo pipefail

source_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)
work_dir=${1:?Pass a working directory for the CI build}
jobs=${CMAKE_BUILD_PARALLEL_LEVEL:-2}
mkdir -p -- "$work_dir"
work_dir=$(cd -- "$work_dir" && pwd -P)
prefix="$work_dir/install"
build_dir="$work_dir/build"

dependencies=$(python3 - "$source_dir/dependencies.repos" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding="utf-8") as manifest:
    repositories = yaml.safe_load(manifest)["repositories"]

for name in ("ros_industrial_cmake_boilerplate", "boost_plugin_loader"):
    repository = repositories[name]
    if repository["type"] != "git":
        raise ValueError(f"Expected a git source for {name}")
    print(name, repository["url"], repository["version"], sep="\t")
PY
)

while IFS=$'\t' read -r name url revision; do
  dep_source="$work_dir/source/$name"
  dep_build="$work_dir/dependencies/$name"
  mkdir -p -- "$dep_source"
  git -C "$dep_source" init --quiet
  git -C "$dep_source" fetch --depth=1 -- "$url" "$revision"
  git -C "$dep_source" checkout --detach FETCH_HEAD
  cmake -S "$dep_source" -B "$dep_build" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$prefix" -DCMAKE_INSTALL_PREFIX="$prefix"
  cmake --build "$dep_build" --parallel "$jobs"
  cmake --install "$dep_build"
done <<< "$dependencies"

cmake -S "$source_dir" -B "$build_dir" -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_PREFIX_PATH="$prefix" \
  -DCALIBRATION_WORKBENCH_BUILD_GUI=ON \
  -DCALIBRATION_WORKBENCH_BUILD_EXAMPLES=ON \
  -DCALIBRATION_WORKBENCH_ENABLE_TESTING=ON \
  -DCALIBRATION_WORKBENCH_ENABLE_RUN_TESTING=OFF \
  -DCALIBRATION_WORKBENCH_ENABLE_CLANG_TIDY=ON
cmake --build "$build_dir" --parallel "$jobs"

(
  cd -- "$build_dir"
  export LD_LIBRARY_PATH="$build_dir/target_finders/opencv:$prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  ctest --output-on-failure --parallel "$jobs" | tee "$work_dir/ctest.log"
  grep -Eq '^[[:space:]]*100% tests passed, 0 tests failed out of [1-9][0-9]*[[:space:]]*$' "$work_dir/ctest.log"
)
