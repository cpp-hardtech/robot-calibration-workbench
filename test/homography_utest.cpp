#include <gtest/gtest.h>
#include <limits>
#include <random>
#include <set>
#include <calibration_workbench/analysis/homography_analysis.h>
#include <calibration_workbench/analysis/projection.h>
#include <calibration_workbench/core/utils.h>
#include <calibration_workbench_tests/observation_creator.h>
#include <calibration_workbench_tests/utilities.h>

using namespace calibration_workbench;

const unsigned TARGET_ROWS = 5;
const unsigned TARGET_COLS = 7;
const double SPACING = 0.025;

class HomographyTest : public ::testing::Test
{
public:
  HomographyTest()
    : camera(test::makeKinectCamera())
    , target(TARGET_ROWS, TARGET_COLS, SPACING)
    , target_to_camera(Eigen::Isometry3d::Identity())
    , sampler(TARGET_ROWS, TARGET_COLS)
  {
    double x = static_cast<double>(TARGET_ROWS - 1) * SPACING / 2.0;
    double y = static_cast<double>(TARGET_COLS - 1) * SPACING / 2.0;
    target_to_camera.translate(Eigen::Vector3d(x, y, 0.5));
    target_to_camera.rotate(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
  }

  test::Camera camera;
  test::Target target;
  Eigen::Isometry3d target_to_camera;
  GridCorrespondenceSampler sampler;
};

TEST_F(HomographyTest, PerfectInitialConditions)
{
  Correspondence2D3D::Set correspondence_set;
  EXPECT_NO_THROW(correspondence_set =
                      test::getCorrespondences(target_to_camera, Eigen::Isometry3d::Identity(), camera, target, true));

  Eigen::VectorXd error = calculateHomographyError(correspondence_set, sampler);
  // Expect all of the errors to be extremely small
  EXPECT_LT(error.maxCoeff(), 1.0e-12);
}

TEST_F(HomographyTest, SwapCorrespondencesConditions)
{
  Correspondence2D3D::Set correspondence_set;
  EXPECT_NO_THROW(correspondence_set =
                      test::getCorrespondences(target_to_camera, Eigen::Isometry3d::Identity(), camera, target, true));

  // Swap the image measurements between 2 arbitrary correspondences
  Correspondence2D3D& c1 = correspondence_set.at(10);
  Eigen::Vector2d in_image_1 = c1.in_image;
  Correspondence2D3D& c2 = correspondence_set.at(21);
  Eigen::Vector2d in_image_2 = c2.in_image;
  c1.in_image = in_image_2;
  c2.in_image = in_image_1;

  Eigen::VectorXd error = calculateHomographyError(correspondence_set, sampler);

  // Expect the error of the points used to create the homography matrix to be very small
  std::vector<std::size_t> sample_indices = sampler.getSampleCorrespondenceIndices();
  for (std::size_t idx : sample_indices)
  {
    EXPECT_LT(error(idx), 1.0e-12);
  }

  // Expect the homography to be off by more than one pixel because of the swap
  EXPECT_GT(error.maxCoeff(), 1.0);
}

TEST_F(HomographyTest, NoisyCorrespondences)
{
  Correspondence2D3D::Set correspondence_set;
  EXPECT_NO_THROW(correspondence_set =
                      test::getCorrespondences(target_to_camera, Eigen::Isometry3d::Identity(), camera, target, true));

  // Add Gaussian noise to the image features
  std::mt19937 mt_rand(RANDOM_SEED);
  const double stdev = 1.0;
  std::normal_distribution<double> dist(0.0, stdev);
  for (Correspondence2D3D& corr : correspondence_set)
  {
    Eigen::Vector2d noise;
    noise << dist(mt_rand), dist(mt_rand);
    corr.in_image += noise;
  }

  Eigen::VectorXd error = calculateHomographyError(correspondence_set, sampler);

  // TODO: create an informed expectation for the max or average error. Initial testing indicates that the average error
  // can occasionally be > 4 * stddev
  //  double stdev_mag = stdev * std::sqrt(2.0);
  EXPECT_GT(error.maxCoeff(), 0.0);
  EXPECT_GT(error.mean(), 0.0);
}

TEST(Homography, RejectsCollinearCorrespondences)
{
  Correspondence2D3D::Set correspondences;
  for (int index = 0; index < 4; ++index)
    correspondences.emplace_back(Eigen::Vector2d(index, 1), Eigen::Vector3d(index, 0, 0));

  EXPECT_THROW(calculateHomography(correspondences), ICalException);
}

TEST(Homography, FitsAllEquationsForNoisyCheckerboard)
{
  Correspondence2D3D::Set correspondences;
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 4; ++col)
    {
      const double x = SPACING * col;
      const double y = SPACING * row;
      Eigen::Vector2d pixel(320.0 + 600.0 * x / 0.9, 240.0 + 610.0 * y / 0.9);
      if (row == 1 && col == 1)
        pixel.x() += 1.5;
      correspondences.emplace_back(pixel, Eigen::Vector3d(x, y, 0.0));
    }

  const auto homography = calculateHomography(correspondences);

  // At the least-squares minimum, the residual is orthogonal to every
  // coefficient column. Accumulate A^T(Ah - b) from the homogeneous projections.
  Eigen::Matrix<double, 8, 1> gradient = Eigen::Matrix<double, 8, 1>::Zero();
  for (const auto& corr : correspondences)
  {
    const Eigen::Vector3d point(corr.in_target.x(), corr.in_target.y(), 1.0);
    const Eigen::Vector3d projected = homography * point;
    const Eigen::Vector2d residual = corr.in_image * projected.z() - projected.head<2>();
    gradient.head<3>() -= residual.x() * point;
    gradient.segment<3>(3) -= residual.y() * point;
    gradient.tail<2>() += corr.in_image.dot(residual) * point.head<2>();
  }
  for (Eigen::Index col = 0; col < gradient.size(); ++col)
    EXPECT_NEAR(gradient(col), 0.0, 1.0e-8) << "coefficient column " << col;
}

TEST(Homography, FitsPerfectCheckerboardWithDistinctRandomSamples)
{
  Correspondence2D3D::Set correspondences;
  for (int row = 0; row < 3; ++row)
    for (int col = 0; col < 4; ++col)
      correspondences.emplace_back(Eigen::Vector2d(320.0 + 20.0 * col, 240.0 + 20.0 * row),
                                   Eigen::Vector3d(SPACING * col, SPACING * row, 0.0));

  const RandomCorrespondenceSampler sampler(correspondences.size(), correspondences.size() / 3, 1);
  const auto indices = sampler.getSampleCorrespondenceIndices();
  SCOPED_TRACE(::testing::PrintToString(indices));
  ASSERT_EQ(indices.size(), 4U);
  EXPECT_EQ(std::set<std::size_t>(indices.begin(), indices.end()).size(), indices.size());
  EXPECT_EQ(sampler.getSampleCorrespondenceIndices(), indices);

  Eigen::VectorXd errors;
  ASSERT_NO_THROW(errors = calculateHomographyError(correspondences, sampler));
  ASSERT_EQ(errors.size(), static_cast<Eigen::Index>(correspondences.size()));
  EXPECT_LT(errors.maxCoeff(), 1.0e-10);
}

TEST(Homography, SamplesEntirePopulationWithoutRepeats)
{
  const RandomCorrespondenceSampler sampler(4, 4, 1);
  const auto indices = sampler.getSampleCorrespondenceIndices();
  EXPECT_EQ(indices.size(), 4U);
  EXPECT_EQ(std::set<std::size_t>(indices.begin(), indices.end()), (std::set<std::size_t>{ 0, 1, 2, 3 }));
}

TEST(CameraBackprojection, RejectsMirroredPlaneBehindCamera)
{
  const CameraIntrinsics intrinsics{ { 600.0, 610.0, 320.0, 240.0 } };
  const Eigen::Isometry3d identity = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d front = identity;
  front.translate(Eigen::Vector3d(0.04, -0.02, 0.85));
  front.rotate(Eigen::AngleAxisd(0.35, Eigen::Vector3d::UnitY()));

  Eigen::Isometry3d behind = identity;
  behind.linear() = front.linear() * Eigen::Vector3d(-1.0, -1.0, 1.0).asDiagonal();
  behind.translation() = -front.translation();

  Observation2D3D observation;
  observation.to_camera_mount = identity;
  observation.to_target_mount = front;
  for (int row = 0; row < 2; ++row)
    for (int col = 0; col < 3; ++col)
    {
      const Eigen::Vector3d target_point(0.025 * col, 0.025 * row, 0.0);
      const Eigen::Vector3d front_point = front * target_point;
      const Eigen::Vector3d behind_point = behind * target_point;
      const Eigen::Vector2d pixel = projectPoint(intrinsics, front_point);
      ASSERT_GT(front_point.z(), 0.0);
      ASSERT_LT(behind_point.z(), 0.0);
      ASSERT_TRUE(pixel.isApprox(projectPoint(intrinsics, behind_point)));
      observation.correspondence_set.emplace_back(pixel, target_point);
    }

  const auto errors = compute3DProjectionError(observation, intrinsics, identity, identity);
  ASSERT_EQ(errors.size(), 6);
  EXPECT_LT(errors.maxCoeff(), 1e-12);

  observation.to_target_mount = behind;
  EXPECT_THROW(compute3DProjectionError(observation, intrinsics, identity, identity), ICalException);
}

TEST(CameraBackprojection, RejectsInvalidCameraMatrixForNonemptyInput)
{
  Eigen::MatrixX2d pixels(1, 2);
  pixels << 320.0, 240.0;
  Eigen::Isometry3d target_in_camera = Eigen::Isometry3d::Identity();
  target_in_camera.translation().z() = 1.0;

  Eigen::Matrix3d calibration;
  calibration << 0.0, 0.0, 320.0, 0.0, 610.0, 240.0, 0.0, 0.0, 1.0;
  EXPECT_THROW(project3D(pixels, target_in_camera, calibration), ICalException);

  calibration(0, 0) = std::numeric_limits<double>::infinity();
  EXPECT_THROW(project3D(pixels, target_in_camera, calibration), ICalException);

  const Eigen::MatrixX2d empty_pixels(0, 2);
  const auto empty_result = project3D(empty_pixels, target_in_camera, calibration);
  EXPECT_EQ(empty_result.rows(), 0);
  EXPECT_EQ(empty_result.cols(), 3);
}

TEST(CameraBackprojection, KeepsBidirectionalLineIntersectionAndRejectsCameraOrigin)
{
  const Eigen::Vector3d behind_point(0.0, 0.0, -2.0);
  const auto intersection = computeLinePlaneIntersection(Eigen::Vector3d::UnitZ(), behind_point,
                                                         Eigen::Vector3d::UnitZ(), Eigen::Vector3d::Zero());
  EXPECT_TRUE(intersection.isApprox(behind_point));

  Eigen::MatrixX2d pixel(1, 2);
  pixel.setZero();
  EXPECT_THROW(project3D(pixel, Eigen::Isometry3d::Identity(), Eigen::Matrix3d::Identity()), ICalException);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
