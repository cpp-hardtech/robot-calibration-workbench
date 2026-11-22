^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package calibration_workbench
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The independent maintenance baseline from 2025-10-02 was recorded as version
1.2.0 and introduced the package name, namespace, include layout, build
options, and documentation.

0.1.0 (2026-11-22)
------------------

* Calibrate camera intrinsics and hand-eye transforms through the C++ SDK and
  Qt applications.
* Run calibration without a display, accept image-only intrinsic observations
  when extrinsic guesses are disabled, and export observation quality as CSV.
* Read and write ROS camera calibration YAML and remap intrinsics to cropped
  or resized images.
* Detect ArUco, ChArUco, checkerboard, and modified circle-grid targets. Export
  annotated images and 2D-to-3D correspondences from the target detection CLI.
* Build the SDK and Docker images on Ubuntu Focal, Jammy, and Noble.
