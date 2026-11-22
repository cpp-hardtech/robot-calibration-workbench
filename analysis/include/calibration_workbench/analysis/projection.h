#pragma once

#include <calibration_workbench/core/camera_intrinsics.h>
#include <calibration_workbench/core/types.h>

#include <Eigen/Dense>

namespace calibration_workbench
{
/**
 * @brief Computes the vector from an input point on a line to the point where the line intersects with the input plane
 * See <a href=https://en.wikipedia.org/wiki/Line%E2%80%93plane_intersection>this link</a> for more information
 * @param plane_normal Unit normal vector defining the plane
 * @param plane_pt Point on the plane
 * @param line Unit vector defining the line
 * @param line_pt Point on the line
 * @param epsilon
 * @ingroup analysis_projection
 */
Eigen::Vector3d computeLinePlaneIntersection(const Eigen::Vector3d& plane_normal, const Eigen::Vector3d& plane_pt,
                                             const Eigen::Vector3d& line, const Eigen::Vector3d& line_pt,
                                             const double epsilon = 1.0e-6);

/**
 * @brief Backprojects image points onto the plane of a flat calibration target
 * @param source Pixel coordinates in the image
 * @param target_in_camera Pose of the target in the camera frame, transforming target points to camera coordinates
 * @param k 3x3 camera intrinsic matrix
 * @return Intersections in target coordinates. An empty input returns an empty matrix.
 * @throws ICalException for a non-finite or singular camera matrix, or an intersection outside the finite forward ray
 * @throws std::runtime_error if an image ray is parallel to the target plane
 * @ingroup analysis_projection
 */
Eigen::MatrixX3d project3D(const Eigen::MatrixX2d& source, const Eigen::Isometry3d& target_in_camera,
                           const Eigen::Matrix3d& k);

/**
 * @brief Projects the 2D target features from an observation onto the 3D plane of the calibrated target and computes
 * the difference between the projections and the known target features
 * @ingroup analysis_projection
 */
Eigen::ArrayXd compute3DProjectionError(const Observation2D3D& obs, const CameraIntrinsics& intr,
                                        const Eigen::Isometry3d& camera_mount_to_camera,
                                        const Eigen::Isometry3d& target_mount_to_target);

/**
 * @brief Projects the 2D target features from a set of observations onto the 3D plane of the calibrated target and
 * computes the difference between the projections and the known target features
 * @ingroup analysis_projection
 */
Eigen::ArrayXd compute3DProjectionError(const Observation2D3D::Set& obs, const CameraIntrinsics& intr,
                                        const Eigen::Isometry3d& camera_mount_to_camera,
                                        const Eigen::Isometry3d& target_mount_to_target);

}  // namespace calibration_workbench
