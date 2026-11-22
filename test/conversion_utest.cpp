#include <gtest/gtest.h>
#include <calibration_workbench/optimizations/ceres_math_utilities.h>
#include <calibration_workbench/core/exceptions.h>
#include <calibration_workbench/core/utils.h>

#include <limits>

using namespace calibration_workbench;

// Test the conversion of a pose from Eigen to internal calibration format and back
TEST(EigenConversions, there_and_back_identity)
{
  const auto e_pose = Eigen::Isometry3d::Identity();
  const auto cal_pose = poseEigenToCal(e_pose);
  const auto recovered_e_pose = poseCalToEigen(cal_pose);

  const Eigen::Matrix4d diff = e_pose.matrix() - recovered_e_pose.matrix();
  EXPECT_TRUE(diff.isZero());
}

// Test the conversion of a pose from Eigen to internal calibration format and back
TEST(EigenConversions, there_and_back)
{
  auto e_pose = Eigen::Isometry3d::Identity();
  e_pose = e_pose * Eigen::Translation3d(0.5, 0.75, 1.0) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitY());
  const auto cal_pose = poseEigenToCal(e_pose);
  const auto recovered_e_pose = poseCalToEigen(cal_pose);

  const Eigen::Matrix4d diff = e_pose.matrix() - recovered_e_pose.matrix();
  EXPECT_TRUE(diff.isZero());
}

TEST(CameraIntrinsics, RemapsProjectionToCroppedAndResizedImage)
{
  const CameraIntrinsics original{ { 800.0, 820.0, 640.0, 360.0 } };
  const Eigen::Vector3d point_in_camera(0.2, -0.1, 2.0);

  const auto remapped = original.remapImageCoordinates(0.5, 0.75, -60.0, -45.0);
  const Eigen::Vector2d pixel = projectPoint(remapped, point_in_camera);

  EXPECT_DOUBLE_EQ(pixel.x(), 300.0);
  EXPECT_DOUBLE_EQ(pixel.y(), 194.25);
}

TEST(CameraIntrinsics, RejectsInvalidCoordinateRemapping)
{
  const CameraIntrinsics intrinsics{ { 800.0, 820.0, 640.0, 360.0 } };
  const double infinity = std::numeric_limits<double>::infinity();

  EXPECT_THROW(intrinsics.remapImageCoordinates(0.0, 1.0), ICalException);
  EXPECT_THROW(intrinsics.remapImageCoordinates(1.0, 1.0, infinity), ICalException);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
