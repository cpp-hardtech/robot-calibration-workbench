#include <gtest/gtest.h>
#include <calibration_workbench/optimizations/ceres_math_utilities.h>
#include <calibration_workbench/optimizations/dh_chain_kinematic_calibration_cost.h>
#include <calibration_workbench/core/exceptions.h>
#include <calibration_workbench/core/utils.h>

#include <ceres/jet.h>
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

TEST(AngleAxisConversion, RetainsRotationDerivativesAtIdentity)
{
  using Jet = ceres::Jet<double, 3>;
  const Eigen::Vector3d point(0.3, -0.4, 0.9);
  const Eigen::Vector3d translation(0.04, -0.02, 0.85);
  const Jet jet_translation[] = { Jet(translation.x()), Jet(translation.y()), Jet(translation.z()) };

  for (const double rotation : { 0.0, 1e-12, 0.35 })
  {
    SCOPED_TRACE(rotation);
    const Jet angle_axis[] = { Jet(rotation, 0), Jet(0.0, 1), Jet(0.0, 2) };
    const Jet* const parameters[] = { jet_translation, angle_axis };
    const Eigen::Vector3d rotated_point = Eigen::AngleAxisd(rotation, Eigen::Vector3d::UnitX()) * point;
    const Eigen::Vector3d expected_point = rotated_point + translation;
    const Eigen::Vector3d expected_x_derivative = Eigen::Vector3d::UnitX().cross(rotated_point);

    for (const auto& transform :
         { toIsometry(angle_axis, jet_translation), DualDHChainCost::createTransform(parameters, 0) })
    {
      const Eigen::Matrix<Jet, 3, 1> transformed = transform * point.cast<Jet>();
      for (Eigen::Index row = 0; row < transformed.size(); ++row)
      {
        EXPECT_NEAR(transformed[row].a, expected_point[row], 1e-12);
        EXPECT_TRUE(transformed[row].v.allFinite());
        EXPECT_NEAR(transformed[row].v[0], expected_x_derivative[row], 1e-12);
      }

      if (rotation == 0.0)
        for (Eigen::Index axis = 0; axis < transformed.size(); ++axis)
        {
          const Eigen::Vector3d expected = Eigen::Vector3d::Unit(axis).cross(point);
          for (Eigen::Index row = 0; row < transformed.size(); ++row)
            EXPECT_DOUBLE_EQ(transformed[row].v[axis], expected[row]);
        }
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
