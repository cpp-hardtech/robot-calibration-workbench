<p align="center">
  <img src=".github/branding/logo.png" alt="Robot Calibration Workbench logo" width="160">
</p>

<h1 align="center">Robot Calibration Workbench</h1>

<p align="center">
  <strong>Calibrate the spatial relationship between an industrial robot and its camera.</strong>
</p>

Robot Calibration Workbench is a C++ toolkit for robot vision calibration. It covers camera
intrinsics, target-based pose estimation, hand-eye calibration, and the analysis needed to
decide whether a calibration result is usable.

The algorithms live in standalone CMake libraries that do not require ROS. The workspace
build and the Qt applications are optional layers on top. Datasets for the hand-eye workflow
ship with the repository, so the examples run without a robot or a camera.

## Components

| Directory | Contents |
| --- | --- |
| `core/` | Data types and shared geometry used by every other component. |
| `optimizations/` | Solvers for camera intrinsics, PnP, hand-eye, and kinematic parameters. |
| `target_finders/` | Target detection. The bundled implementation uses OpenCV. |
| `analysis/` | Reprojection checks, pose differences, and uncertainty analysis. |
| `gui/` | Qt applications for camera intrinsic and hand-eye calibration. |
| `examples/` | Runnable examples, including a hand-eye workflow on bundled data. |

## Checkerboard target

Use this target finder with a plain checkerboard in a camera calibration dataset. Specify
the number of inner corners across each dimension and the spacing between corners in meters.
Use images in which the complete checkerboard is visible.

```yaml
target_finder:
  type: ChessboardGridTargetFinder
  rows: 5
  cols: 7
  spacing: 0.03
```

## Requirements

- Linux with a C++17-capable toolchain and CMake.
- OpenCV for the target finder and the examples.
- Qt for the GUI applications.
- ROS 2, `vcs`, `colcon`, and an initialised `rosdep` for the workspace build. The core
  libraries also build as a plain CMake package without ROS.

## Build

Create a workspace, pull the declared dependencies, and build with `colcon`:

```bash
mkdir -p calibration-ws/src
cd calibration-ws
git clone https://github.com/cpp-hardtech/robot-calibration-workbench.git src/calibration_workbench
vcs import src < src/calibration_workbench/dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

The GUI applications and the examples are controlled by
`CALIBRATION_WORKBENCH_BUILD_GUI` and `CALIBRATION_WORKBENCH_BUILD_EXAMPLES` in the top-level
`CMakeLists.txt`; both default to `ON`. Testing and clang-tidy are behind
`CALIBRATION_WORKBENCH_ENABLE_TESTING` and `CALIBRATION_WORKBENCH_ENABLE_CLANG_TIDY`.

### ROS camera calibration YAML

Use `calibration_workbench::fromROSFormat(YAML::LoadFile(path))` from
`calibration_workbench/optimizations/camera_intrinsic.h` to load a ROS pinhole camera calibration with
`plumb_bob` distortion. The result contains the camera name, image dimensions, intrinsics, and distortion
coefficients.

### Camera intrinsics after cropping or resizing

`CameraIntrinsics::remapImageCoordinates` returns calibration parameters in the new image coordinates.
For an image cropped at `(left, top)` and then resized, pass
`(scale_x, scale_y, -scale_x * left, -scale_y * top)`. The principal point can lie outside the cropped image.

## Run the hand-eye example

The example needs a graphical session. Keep the source checkout in place, because the example
loads its dataset from a path recorded at build time.

```bash
source install/setup.bash
./install/calibration_workbench/lib/calibration_workbench/calibration_workbench_extrinsic_hand_eye
```

It runs against a modified circle grid and a ChArUco grid, prints the estimated transforms and
analysis statistics, and displays the target points reprojected into the input images.

The bundled data is enough to run it without hardware. See
[examples/extrinsic_hand_eye.cpp](examples/extrinsic_hand_eye.cpp) for the source and
[examples/data/](examples/data/) for the datasets.

## Run calibration without a display

Start either calibration application with `--headless` followed by its configuration, observation,
and result files. The camera intrinsic application also accepts an optional ROS YAML output file.
Append `--observation-report report.csv` to either command to save the observation report alongside
the calibration result. Use a separate path for the report.

```bash
calibration_workbench_camera_intrinsic_calibration_app --headless config.yaml observations.yaml results.yaml ros-results.yaml
calibration_workbench_extrinsic_hand_eye_calibration_app --headless config.yaml observations.yaml results.yaml --observation-report report.csv
```

For intrinsic calibration, set `use_extrinsic_guesses: false` in the configuration file to use observations
with an `image` entry and no `pose` entry. Hand-eye calibration and intrinsic calibration with extrinsic
guesses require both files.

The applications exit after saving a converged result and report calibration errors to standard error.

## Docker

A container definition and a Compose file are provided under `docker/`. The Compose file
builds the image from this repository and mounts the X11 socket so the GUI applications can
start.

```bash
cd docker
docker compose build
docker compose run --rm calibration_workbench
```

## Inspect target detections

Read a target configuration and one image without opening a display. Pass a standalone target
configuration or a calibration YAML with a `target_finder` entry. After sourcing the workspace
environment, run:

```bash
./install/calibration_workbench/lib/calibration_workbench/calibration_workbench_detect_target \
  cal_data.yaml image.png > detections.csv
```

The default CSV contains `feature_id,point_index,x,y` with image coordinates in pixels.
Use `--overlay` to save an annotated image and `--correspondences` to export matched 2D
image points and 3D target points:

```bash
./install/calibration_workbench/lib/calibration_workbench/calibration_workbench_detect_target \
  cal_data.yaml image.png --overlay marked.png --correspondences > correspondences.csv
```

Use an image extension such as `.png` for the overlay and an output path different from
the input files. Correspondence CSV columns are `image_x,image_y,target_x,target_y,target_z`.
Image points are in pixels. Target points use the units defined in the target configuration.
The command returns a nonzero status if detection fails or a requested output cannot be written.

### Batch target diagnostics

Pass `--batch` after the target configuration, followed by one or more image paths:

```bash
./install/calibration_workbench/lib/calibration_workbench/calibration_workbench_detect_target \
  cal_data.yaml --batch 'frame "left", 01.png' 'frame right 02.png' > batch.csv
```

The CSV header is `image_path,status,feature_count,point_count`. Every input has one row,
in argument order. Paths are quoted with embedded double quotes doubled. `feature_count`
counts detected feature IDs and `point_count` counts all detected image points. Status is
`detected`, `no_detection`, `unreadable`, or `detector_error`. A failed input has zero
counts, and the command continues with the remaining inputs. Use `--overlay` and
`--correspondences` in single-image mode.

Exit status is `0` when every image has detections, `1` when any image fails or a configuration
or output error occurs, and `2` for invalid usage. A configuration error is reported before
the command writes any CSV.

## Documentation

- [Getting started](docs/page_getting_started.dox)
- [Camera intrinsic calibration](docs/page_camera_intrinsic_calibration.dox)
- [Extrinsic hand-eye calibration](docs/page_extrinsic_hand_eye_calibration.dox)

## Export observation quality

In either calibration application, choose **File → Export observation report** to save the
current observations as CSV. The report includes image and pose paths, detected feature
counts, homography errors in pixels, and the status shown for each observation.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md).

## License

Distributed under the Apache License, Version 2.0. The original copyright notice and the
full license text are retained in [LICENSE](LICENSE). Provenance and attribution are
documented in [NOTICE](NOTICE).

In accordance with section 4(b) of the License, modification notices are recorded in the
repository history and in NOTICE.
