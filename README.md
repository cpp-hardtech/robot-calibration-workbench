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

The camera intrinsic application also reads this file from its configuration:

```yaml
intrinsics_guess:
  ros_camera_info: cameras/camera.yaml
```

Use the same `ros_camera_info` entry under `intrinsics` in the hand-eye application. The intrinsic
application also accepts `intrinsics` as an alias for `intrinsics_guess`. Paths are relative to the
application configuration file. This entry replaces the inline `fx`, `fy`, `cx`, and `cy` fields.
Both applications use these four pinhole values from the ROS camera matrix. The distortion and
rectification values are omitted from the application's camera model.

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

### Validate observations before calibration

Use `--validate-only` to check images and poses, detect features, and apply the calibration's
homography threshold. The command writes the same observation-quality CSV as calibration.

```bash
calibration_workbench_camera_intrinsic_calibration_app --validate-only config.yaml observations.yaml report.csv
calibration_workbench_extrinsic_hand_eye_calibration_app --validate-only config.yaml observations.yaml report.csv
```

The command exits with 0 when it writes the report with at least one usable observation. If every
observation is rejected, it writes the rejection reasons and exits with 1. Invalid or empty input
and report-write errors also exit with 1. Validation does not solve a calibration or write result files.

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

## Documentation

- [Getting started](docs/page_getting_started.dox)
- [Camera intrinsic calibration](docs/page_camera_intrinsic_calibration.dox)
- [Extrinsic hand-eye calibration](docs/page_extrinsic_hand_eye_calibration.dox)

## Build an observation manifest

Create the observations YAML for a calibration application from an image directory. Add
`--poses` to pair images with robot poses for hand-eye calibration or intrinsic
calibration with extrinsic guesses:

```bash
prefix=install/calibration_workbench
dataset=/path/to/dataset
"$prefix/lib/calibration_workbench/calibration_workbench_make_observations" \
  "$dataset/images" --poses "$dataset/poses" --relative-to "$dataset" > "$dataset/observations.yaml"
```

The command writes `data` entries with an `image` path and, when requested, a `pose` path.
The default paths are absolute. With `--relative-to`, save the manifest in the specified
existing directory so the calibration applications can resolve the relative paths.

The scan includes regular files directly in each directory. Supported image suffixes are
`.bmp`, `.jpeg`, `.jpg`, `.pbm`, `.pgm`, `.png`, `.pnm`, `.ppm`, `.tif`, `.tiff`, and
`.webp`. Pose suffixes are `.yaml` and `.yml`. Suffix matching is case-insensitive. Image
entries are sorted lexicographically by filename stem, and pose matching uses the exact,
case-sensitive stem. The generator uses file names and leaves image and pose validation
to the calibration application.

Exit status is `0` for a manifest, `1` for a directory, pairing, or output error, and `2`
for invalid usage. Empty scans, duplicate stems, and missing pose matches produce a
diagnostic on standard error before the command writes a manifest.

## Export observation quality

In either calibration application, choose **File → Export observation report** to save the
current observations as CSV. The report includes image and pose paths, detected feature
counts, homography errors in pixels, and the status shown for each observation.

## Estimate a target pose

Estimate the pose of a configured target in a single image without opening a display.
Pass a standalone target configuration or a calibration YAML with a `target_finder`
entry, a ROS `plumb_bob` camera calibration YAML, and an image of the calibrated
resolution. The bundled ChArUco sample contains all three inputs:

```bash
source install/setup.bash
prefix=install/calibration_workbench
sample="$prefix/share/calibration_workbench/data/test_set_charuco"
"$prefix/lib/calibration_workbench/calibration_workbench_estimate_target_pose" \
  "$sample/cal_data.yaml" "$sample/camera.ros.yaml" "$sample/images/0.png" > pose.yaml
```

The YAML contains `target_to_camera` (translation `x,y,z` and quaternion `qw,qx,qy,qz`),
`correspondence_count`, and `reprojection_rms_px`. The transform maps a target-frame point
into the OpenCV camera optical frame (right, down, forward): `p_camera = R * p_target + t`.
Translation uses the target's configured units. RMS is the root mean square Euclidean
reprojection error per point in pixels, using the calibrated distortion coefficients.

Exit status is `0` for a pose, `2` for invalid command usage, and `1` for invalid inputs,
insufficient target correspondences, or pose estimation or output failure. Input and
estimation errors are reported to standard error before writing a result.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md).

## License

Distributed under the Apache License, Version 2.0. The original copyright notice and the
full license text are retained in [LICENSE](LICENSE). Provenance and attribution are
documented in [NOTICE](NOTICE).

In accordance with section 4(b) of the License, modification notices are recorded in the
repository history and in NOTICE.
