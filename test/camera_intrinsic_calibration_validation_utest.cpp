#include <gtest/gtest.h>
#include <calibration_workbench/optimizations/camera_intrinsic.h>
#include <calibration_workbench/analysis/camera_intrinsic_calibration_analysis.h>
#include <calibration_workbench/analysis/extrinsic_hand_eye_calibration_analysis.h>
#include <calibration_workbench/optimizations/extrinsic_hand_eye.h>
#include <calibration_workbench/optimizations/pnp.h>
#include <calibration_workbench/core/serialization.h>
#include <calibration_workbench/core/utils.h>
#include <calibration_workbench_tests/utilities.h>
#include <calibration_workbench_tests/observation_creator.h>
#include <array>
#include <cmath>
#include <iostream>
#include <memory>

using namespace calibration_workbench;

TEST(CameraIntrinsicCalibrationValidation, RejectsEmptyImageObservations)
{
  CameraIntrinsicProblem problem{};
  EXPECT_THROW(optimize(problem), OptimizationException);

  problem.image_observations.resize(1);
  problem.extrinsic_guesses.push_back(Eigen::Isometry3d::Identity());
  problem.use_extrinsic_guesses = true;
  EXPECT_THROW(optimize(problem), OptimizationException);
}

TEST(CameraIntrinsicCalibrationValidation, RejectsMismatchedExtrinsicGuessCount)
{
  CameraIntrinsicProblem problem{};
  problem.intrinsics_guess = CameraIntrinsics{ { 600.0, 610.0, 320.0, 240.0 } };
  problem.use_extrinsic_guesses = true;
  problem.image_observations.resize(1);
  problem.image_observations.front().emplace_back(Eigen::Vector2d(320.0, 240.0), Eigen::Vector3d(0.0, 0.0, 1.0));

  EXPECT_THROW(optimize(problem), OptimizationException);

  problem.extrinsic_guesses = { Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity() };
  EXPECT_THROW(optimize(problem), OptimizationException);
}

TEST(CameraIntrinsicCalibrationValidation, MeasureVirtualTargetDiff)
{
  test::Target target(8, 6, 0.025);
  test::Camera camera = test::makeKinectCamera();

  // Create a camera pose directly above the target
  Eigen::Isometry3d base_to_target(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d base_to_camera =
      base_to_target * Eigen::Translation3d(0.0, 0.0, 1.0) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());

  Correspondence2D3D::Set corr_set;
  ASSERT_NO_THROW(corr_set = test::getCorrespondences(base_to_camera, base_to_target, camera, target, true));

  Eigen::Isometry3d camera_to_target = base_to_camera.inverse() * base_to_target;

  /* Since `getInternalTargetTransformation` simply splits the correspondences in half,
   * creating two virtual targets with different features relative to the same origin,
   * the expected transformation from one virtual target to the other should be identity */
  const Eigen::Isometry3d expected(Eigen::Isometry3d::Identity());

  // Perfect camera intrinsics, perfect camera to target guess
  {
    VirtualCorrespondenceResult result;
    EXPECT_NO_THROW(result = measureVirtualTargetDiff(corr_set, camera.intr, camera_to_target));

    // Expect the transform to be almost exactly as expected
    EXPECT_TRUE(result.t1_to_t2.isApprox(expected));
  }

  // Create a slightly noisy camera to target guess (1 cm positional and ~5 degrees orientation noise)
  Eigen::Isometry3d camera_to_target_guess = test::perturbPose(camera_to_target, 0.01, 0.01);

  // Perfect camera intrinsics, imperfect camera to target guess
  {
    VirtualCorrespondenceResult result;
    EXPECT_NO_THROW(result = measureVirtualTargetDiff(corr_set, camera.intr, camera_to_target_guess));

    // Expect the transform to be very close to its expected value, but with a small error from the PnP
    EXPECT_TRUE(result.t1_to_t2.isApprox(expected, 1.0e-8));
  }

  // Bad camera intrinsics, imperfect camera to target guess
  {
    // Tweak the camera intrinsics slightly (+/- 1%)
    camera.intr.fx() += 0.01 * camera.intr.fx();
    camera.intr.fy() -= 0.01 * camera.intr.fy();
    camera.intr.cx() -= 0.01 * camera.intr.cx();
    camera.intr.cy() += 0.01 * camera.intr.cy();

    VirtualCorrespondenceResult result;
    EXPECT_NO_THROW(result = measureVirtualTargetDiff(corr_set, camera.intr, camera_to_target_guess));

    // Expect the transform to not be very close to its expected value
    EXPECT_FALSE(result.t1_to_t2.isApprox(expected, 1.0e-8));
  }
}

TEST(CameraIntrinsicCalibrationValidation, MeasuresVirtualTargetsOnTwoRowCheckerboard)
{
  const CameraIntrinsics intrinsics{ { 600.0, 610.0, 320.0, 240.0 } };
  Eigen::Isometry3d camera_to_target = Eigen::Isometry3d::Identity();
  camera_to_target.translation().z() = 1.0;

  Correspondence2D3D::Set correspondences;
  for (int row = 0; row < 2; ++row)
    for (int col = 0; col < 4; ++col)
    {
      const Eigen::Vector3d target(0.025 * col, 0.025 * row, 0.0);
      const Eigen::Vector3d camera_point = camera_to_target * target;
      correspondences.emplace_back(projectPoint(intrinsics, camera_point), target);
    }

  std::cout << "Virtual target fixture: 2 rows, 4 columns, row-major, depth 1 m\n";
  VirtualCorrespondenceResult result;
  ASSERT_NO_THROW(result = measureVirtualTargetDiff(correspondences, intrinsics, camera_to_target));
  EXPECT_NEAR(result.positional_error, 0.0, 1e-10);
  EXPECT_NEAR(result.angular_error, 0.0, 1e-10);
}

TEST(CameraIntrinsicCalibrationValidation, UsesMeanSquaredPixelErrorForVirtualTargetLimit)
{
  const CameraIntrinsics intrinsics{ { 600.0, 610.0, 320.0, 240.0 } };
  Eigen::Isometry3d camera_to_target = Eigen::Isometry3d::Identity();
  camera_to_target.translation().z() = 1.0;

  // The pattern is orthogonal to constant, linear, and quadratic functions of
  // the column position. The reprojection error cannot be removed locally by
  // translating, rotating, or tilting the target.
  const std::array<double, 4> column_pattern{ 1.0, -3.0, 3.0, -1.0 };
  const double amplitude = 1.5 / std::sqrt(5.0);
  Correspondence2D3D::Set correspondences;
  for (int row = 0; row < 4; ++row)
    for (std::size_t col = 0; col < column_pattern.size(); ++col)
    {
      const Eigen::Vector3d target(0.025 * col, 0.025 * row, 0.0);
      const Eigen::Vector3d camera_point = camera_to_target * target;
      Eigen::Vector2d image = projectPoint(intrinsics, camera_point);
      image.x() += amplitude * column_pattern[col];
      correspondences.emplace_back(image, target);
    }

  for (std::size_t half = 0; half < 2; ++half)
  {
    PnPProblem problem;
    problem.intr = intrinsics;
    problem.camera_to_target_guess = camera_to_target;
    const auto begin = correspondences.begin() + half * 8;
    problem.correspondences.assign(begin, begin + 8);
    const auto solution = optimize(problem);
    ASSERT_TRUE(solution.converged);
    double sum_squared_error = 0.0;
    for (const auto& corr : problem.correspondences)
    {
      const Eigen::Vector3d camera_point = solution.camera_to_target * corr.in_target;
      sum_squared_error += (projectPoint(intrinsics, camera_point) - corr.in_image).squaredNorm();
    }
    const double mean_squared_error = sum_squared_error / problem.correspondences.size();
    std::cout << "Virtual target half " << half << ": " << problem.correspondences.size()
              << " features, solver cost " << solution.final_cost_per_obs << ", mean squared error "
              << mean_squared_error << " px^2\n";
    ASSERT_NEAR(mean_squared_error, 2.25, 1e-8);
  }

  EXPECT_THROW(measureVirtualTargetDiff(correspondences, intrinsics, camera_to_target, 1.0), ICalException);
  EXPECT_NO_THROW(measureVirtualTargetDiff(correspondences, intrinsics, camera_to_target, 3.0));
}

TEST(CameraIntrinsicCalibrationValidation, MeasureIntrinsicCalibrationAccuracy)
{
  test::Camera camera = test::makeKinectCamera();
  test::Target target(5, 7, 0.025);
  auto pose_gen = std::make_shared<test::GridPoseGenerator>();

  // Create the relevant calibration transforms
  Eigen::Isometry3d camera_mount_to_camera(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d target_mount_to_target(Eigen::Isometry3d::Identity());
  Eigen::Isometry3d camera_base_to_target_base(Eigen::Isometry3d::Identity());

  // Create some observations
  Observation2D3D::Set observations =
      test::createObservations(camera, target, { pose_gen }, target_mount_to_target, camera_mount_to_camera);

  // Validate the intrinsic calibration
  // Perfect intrinsic parameters, perfect transform guesses
  {
    IntrinsicCalibrationAccuracyResult result = measureIntrinsicCalibrationAccuracy(
        observations, camera.intr, camera_mount_to_camera, target_mount_to_target, camera_base_to_target_base);

    // Expect results to be within 2 sigma (~95%) of the threshold
    {
      double threshold = 1.0e-13;
      EXPECT_LT(result.pos_error.first + 2.0 * result.pos_error.second, threshold);
    }
    {
      double threshold = 1.0e-13;
      EXPECT_LT(result.ang_error.first + 2.0 * result.ang_error.second, threshold);
    }
  }

  // Perturb the calibration transforms slightly
  Eigen::Isometry3d camera_mount_to_camera_guess = test::perturbPose(camera_mount_to_camera, 0.01, 0.01);
  Eigen::Isometry3d target_mount_to_target_guess = test::perturbPose(target_mount_to_target, 0.01, 0.01);

  // Perfect intrinsic parameters, imperfect transform guesses
  {
    IntrinsicCalibrationAccuracyResult result =
        measureIntrinsicCalibrationAccuracy(observations, camera.intr, camera_mount_to_camera_guess,
                                            target_mount_to_target_guess, camera_base_to_target_base);

    // Expect results to be within 2 sigma (~95%) of the threshold
    {
      double threshold = 1.0e-7;
      EXPECT_LT(result.pos_error.first + 2.0 * result.pos_error.second, threshold);
    }
    {
      double threshold = 1.0e-7;
      EXPECT_LT(result.ang_error.first + 2.0 * result.ang_error.second, threshold);
    }
  }

  // Imperfect intrinsic parameters, imperfect transform guesses
  {
    // Changing the intrinsic values by 1% should yield an error of more than 1mm
    camera.intr.fx() += 0.01 * camera.intr.fx();
    camera.intr.fy() -= 0.01 * camera.intr.fy();
    camera.intr.cx() -= 0.01 * camera.intr.cx();
    camera.intr.cx() += 0.01 * camera.intr.cy();

    IntrinsicCalibrationAccuracyResult result =
        measureIntrinsicCalibrationAccuracy(observations, camera.intr, camera_mount_to_camera_guess,
                                            target_mount_to_target_guess, camera_base_to_target_base);

    // Expect results not to be within 2 sigma (~95%) of the threshold because we modified the camera intrinsics
    {
      double threshold = 1.0e-3;
      EXPECT_GT(result.pos_error.first + 2.0 * result.pos_error.second, threshold);
    }
    {
      double threshold = 1.0e-3;
      EXPECT_GT(result.ang_error.first + 2.0 * result.ang_error.second, threshold);
    }
  }
}

TEST(ROSCameraCalibration, ImportsExportedCalibration)
{
  CameraIntrinsicResult original{};
  original.intrinsics = CameraIntrinsics{ { 743.0, 752.0, 631.5, 357.0 } };
  original.distortions = { { -0.21, 0.08, 0.002, -0.002, 0.001 } };
  YAML::Node document = toROSFormat(original, 1280, 720);
  document["camera_name"] = "left";

  const auto parsed = fromROSFormat(YAML::Load(YAML::Dump(document)));
  EXPECT_EQ(parsed.camera_name, "left");
  EXPECT_EQ(parsed.image_width, 1280);
  EXPECT_EQ(parsed.image_height, 720);
  EXPECT_EQ(parsed.intrinsics, original.intrinsics);
  EXPECT_EQ(parsed.distortions, original.distortions);
}

TEST(ROSCameraCalibration, RejectsUnsupportedOrMalformedCalibration)
{
  CameraIntrinsicResult original{};
  original.intrinsics = CameraIntrinsics{ { 743.0, 752.0, 631.5, 357.0 } };
  YAML::Node document = toROSFormat(original, 1280, 720);

  document["distortion_model"] = "equidistant";
  EXPECT_THROW(fromROSFormat(document), BadFileException);
  document["distortion_model"] = "plumb_bob";

  document["camera_matrix"]["rows"] = 4;
  EXPECT_THROW(fromROSFormat(document), BadFileException);
  document["camera_matrix"]["rows"] = 3;

  document["distortion_coefficients"]["data"].push_back(0.0);
  EXPECT_THROW(fromROSFormat(document), BadFileException);
}

TEST(CameraIntrinsicCalibrationValidation, CalibratesCroppedImageWithPrincipalPointOutsideCrop)
{
  const CameraIntrinsics full_image{ { 800.0, 810.0, 640.0, 360.0 } };
  const Eigen::Vector2d crop_origin(690.0, 390.0);
  const Eigen::Vector2d scale(0.8, 0.7);

  CameraIntrinsicProblem problem{};
  problem.intrinsics_guess = full_image.remapImageCoordinates(scale.x(), scale.y(), -scale.x() * crop_origin.x(),
                                                              -scale.y() * crop_origin.y());
  problem.use_extrinsic_guesses = true;

  for (int view = 0; view < 10; ++view)
  {
    Eigen::Isometry3d camera_to_target = Eigen::Isometry3d::Identity();
    camera_to_target.translate(
        Eigen::Vector3d(0.10 + 0.007 * (view % 5), 0.065 + 0.008 * (view % 3), 0.42 + 0.027 * (view % 4)));
    camera_to_target.rotate(Eigen::AngleAxisd(0.10 * (view % 5 - 2), Eigen::Vector3d::UnitX()));
    camera_to_target.rotate(Eigen::AngleAxisd(0.12 * (view % 4 - 1), Eigen::Vector3d::UnitY()));
    problem.extrinsic_guesses.push_back(camera_to_target);

    auto& observations = problem.image_observations.emplace_back();
    for (int row = 0; row < 6; ++row)
      for (int col = 0; col < 8; ++col)
      {
        const Eigen::Vector3d target_point(0.025 * col, 0.025 * row, 0.0);
        const Eigen::Vector3d camera_point = camera_to_target * target_point;
        const Eigen::Vector2d full_pixel = projectPoint(full_image, camera_point);
        observations.emplace_back((full_pixel - crop_origin).cwiseProduct(scale), target_point);
      }
  }

  const auto result = optimize(problem);

  EXPECT_TRUE(result.converged);
  EXPECT_NEAR(result.intrinsics.cx(), -40.0, 1e-8);
  EXPECT_NEAR(result.intrinsics.cy(), -21.0, 1e-8);
  EXPECT_LT(result.final_cost_per_obs, 1e-18);
}

TEST(CameraIntrinsicCalibrationValidation, RoundTripsIntrinsicProblemAndLoadsLegacyKey)
{
  CameraIntrinsicProblem problem{};
  problem.intrinsics_guess = CameraIntrinsics{ { 640.0, 567.0, -40.0, -21.0 } };
  YAML::Node document = YAML::Load(YAML::Dump(YAML::Node(problem)));

  ASSERT_TRUE(document["intrinsics_guess"].IsDefined());
  EXPECT_FALSE(document["intrisics_guess"].IsDefined());
  EXPECT_EQ(document.as<CameraIntrinsicProblem>().intrinsics_guess, problem.intrinsics_guess);

  document["intrisics_guess"] = YAML::Clone(document["intrinsics_guess"]);
  document.remove("intrinsics_guess");
  EXPECT_EQ(document.as<CameraIntrinsicProblem>().intrinsics_guess, problem.intrinsics_guess);
}

static Observation2D3D makeAnalysisObservation(const CameraIntrinsics& intrinsics)
{
  Observation2D3D observation;
  observation.to_camera_mount = Eigen::Isometry3d::Identity();
  observation.to_target_mount = Eigen::Isometry3d::Identity();
  observation.to_target_mount.translate(Eigen::Vector3d(0.04, -0.02, 0.85));
  observation.to_target_mount.rotate(Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitX()));

  for (int row = 0; row < 4; ++row)
    for (int col = 0; col < 5; ++col)
    {
      const Eigen::Vector3d target_point(0.02 * col, 0.02 * row, 0.0);
      const Eigen::Vector3d camera_point = observation.to_target_mount * target_point;
      observation.correspondence_set.emplace_back(projectPoint(intrinsics, camera_point), target_point);
    }
  return observation;
}

TEST(CalibrationAnalysisInputValidation, RejectsInsufficientIntrinsicAccuracyObservations)
{
  const CameraIntrinsics intrinsics{ { 600.0, 610.0, 320.0, 240.0 } };
  const auto identity = Eigen::Isometry3d::Identity();
  Observation2D3D::Set observations;

  EXPECT_THROW(measureIntrinsicCalibrationAccuracy(observations, intrinsics, identity, identity, identity),
               ICalException);
  observations.push_back(makeAnalysisObservation(intrinsics));
  EXPECT_THROW(measureIntrinsicCalibrationAccuracy(observations, intrinsics, identity, identity, identity),
               ICalException);
}

TEST(CalibrationAnalysisInputValidation, RejectsInsufficientHandEyeAnalysisObservations)
{
  ExtrinsicHandEyeProblem2D3D problem{};
  problem.intr = CameraIntrinsics{ { 600.0, 610.0, 320.0, 240.0 } };
  ExtrinsicHandEyeResult result{};
  result.camera_mount_to_camera = Eigen::Isometry3d::Identity();
  result.target_mount_to_target = Eigen::Isometry3d::Identity();

  EXPECT_THROW(analyzeResults(problem, result), ICalException);
  problem.observations.push_back(makeAnalysisObservation(problem.intr));
  EXPECT_THROW(analyzeResults(problem, result), ICalException);

  problem.observations.push_back(problem.observations.front());
  EXPECT_NEAR(analyzeResults(problem, result).pos_diff_stdev, 0.0, 1e-10);
}

TEST(CalibrationAnalysisInputValidation, RequiresTwoReprojectionErrors)
{
  ExtrinsicHandEyeProblem2D3D problem{};
  problem.intr = CameraIntrinsics{ { 600.0, 610.0, 320.0, 240.0 } };
  ExtrinsicHandEyeResult result{};
  result.camera_mount_to_camera = Eigen::Isometry3d::Identity();
  result.target_mount_to_target = Eigen::Isometry3d::Identity();

  EXPECT_THROW(analyze3dProjectionError(problem, result), ICalException);

  const auto observation = makeAnalysisObservation(problem.intr);
  problem.observations.push_back(observation);
  problem.observations.front().correspondence_set.resize(1);
  EXPECT_THROW(analyze3dProjectionError(problem, result), ICalException);

  problem.observations.front().correspondence_set.push_back(observation.correspondence_set[1]);
  EXPECT_NEAR(analyze3dProjectionError(problem, result).stdev, 0.0, 1e-10);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
