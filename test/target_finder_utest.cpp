#include <calibration_workbench/target_finders/opencv/target_finder.h>
// Implementations
#include <calibration_workbench/target_finders/opencv/modified_circle_grid_target_finder.h>
#include <calibration_workbench/target_finders/opencv/charuco_grid_target_finder.h>
#include <calibration_workbench/target_finders/opencv/chessboard_grid_target_finder.h>
#include <calibration_workbench/target_finders/opencv/aruco_grid_target_finder.h>
// Analysis
#include <calibration_workbench/analysis/homography_analysis.h>

#include <calibration_workbench/core/exceptions.h>
#include <gtest/gtest.h>
#include <limits>
#include <memory>
#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

using namespace calibration_workbench;

TEST(ChessboardGridFinderTest, createsPlanarCorrespondencesFromPrintedCheckerboard)
{
  constexpr int rows = 5;
  constexpr int cols = 7;
  constexpr int tile = 40;
  constexpr double spacing = 0.03;
  const ChessboardGridTargetFinder finder(ChessboardGridTarget(rows, cols, spacing));
  cv::Mat image((rows + 3) * tile, (cols + 3) * tile, CV_8UC1, cv::Scalar(255));
  for (int row = 0; row <= rows; ++row)
    for (int col = 0; col <= cols; ++col)
      if ((row + col) % 2 == 0)
        image(cv::Rect((col + 1) * tile, (row + 1) * tile, tile, tile)).setTo(cv::Scalar(0));

  const auto features = finder.findTargetFeatures(image);
  ASSERT_EQ(features.size(), rows * cols);
  const auto correspondences = finder.target().createCorrespondences(features);
  ASSERT_EQ(correspondences.size(), rows * cols);
  EXPECT_TRUE(correspondences.front().in_target.isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(
      correspondences.back().in_target.isApprox(Eigen::Vector3d((cols - 1) * spacing, (rows - 1) * spacing, 0.0)));
  EXPECT_NEAR((correspondences[1].in_image - correspondences[0].in_image).norm(), tile, 0.5);
  const cv::Mat blank(image.size(), image.type(), cv::Scalar(255));
  EXPECT_TRUE(finder.findTargetFeatures(blank).empty());
}

class TargetFinderTestFixture : public ::testing::Test
{
public:
  TargetFinderTestFixture() : ::testing::Test(), homography_error_threshold(1.0) {}

  void runTest()
  {
    // Load the test image
    cv::Mat m = cv::imread(filename);
    ASSERT_FALSE(m.empty());

    // Find the target features
    TargetFeatures2D target_features = this->finder->findTargetFeatures(m);

    ASSERT_EQ(target_features.size(), n_expected_ids);

    // Check the homography transform between the points in the image and the target points
    // to make sure the correspondences were identified correctly
    if (corr_sampler)
    {
      auto corrs = finder->target().createCorrespondences(target_features);
      Eigen::VectorXd err = calculateHomographyError(corrs, *corr_sampler);
      std::cout << "Homography Error" << std::endl;
      std::cout << "Min: " << err.array().minCoeff() << std::endl;
      std::cout << "Max: " << err.array().maxCoeff() << std::endl;
      std::cout << "Mean: " << err.array().mean() << std::endl;

      EXPECT_LT(err.array().mean(), homography_error_threshold);
    }
  }

  void verifyOverlayPreservesInput()
  {
    const cv::Mat input = cv::imread(filename);
    ASSERT_FALSE(input.empty());
    const cv::Mat original = input.clone();
    const auto features = finder->findTargetFeatures(input);
    ASSERT_FALSE(features.empty());

    const cv::Mat overlay = finder->drawTargetFeatures(input, features);

    EXPECT_EQ(cv::norm(input, original, cv::NORM_INF), 0.0);
    EXPECT_GT(cv::norm(overlay, original, cv::NORM_INF), 0.0);
  }

  std::shared_ptr<TargetFinderOpenCV> finder;
  std::string filename;
  std::size_t n_expected_ids;
  std::shared_ptr<CorrespondenceSampler> corr_sampler;
  double homography_error_threshold;
};

class ModifiedCircleGridFinderTest : public TargetFinderTestFixture
{
public:
  ModifiedCircleGridFinderTest() : TargetFinderTestFixture(), target(10, 10, 0.025)
  {
    finder = std::make_shared<ModifiedCircleGridTargetFinder>(target);
  }

  virtual void SetUp() override
  {
    filename = std::string(TEST_SUPPORT_DIR) + "modified_circle_grid_10x10_0.025.png";

    // Expect all corners to be detected
    n_expected_ids = target.rows * target.cols;

    // Set up the homography check correspondence sampler
    corr_sampler = std::make_shared<GridCorrespondenceSampler>(target.rows, target.cols, 1);
    homography_error_threshold = 1.0;
  }

  const ModifiedCircleGridTarget target;
};

TEST(CircleDetectorImageTest, detectsSameKeypointsInBgrAndBgraImages)
{
  const cv::Mat bgr = cv::imread(std::string(TEST_SUPPORT_DIR) + "modified_circle_grid_10x10_0.025.png");
  ASSERT_FALSE(bgr.empty());
  cv::Mat bgra;
  cv::cvtColor(bgr, bgra, cv::COLOR_BGR2BGRA);
  CircleDetector detector(CircleDetectorParams{});
  std::vector<cv::KeyPoint> expected, actual;

  detector.detect(bgr, expected);
  detector.detect(bgra, actual);

  ASSERT_FALSE(expected.empty());
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i)
  {
    EXPECT_EQ(actual[i].pt, expected[i].pt);
    EXPECT_EQ(actual[i].size, expected[i].size);
  }
  EXPECT_FALSE(detector.drawDetectedCircles(bgra).empty());
}

class CharucoFinderTest : public TargetFinderTestFixture
{
public:
  CharucoFinderTest() : TargetFinderTestFixture(), target(7, 5, 0.02f, 0.01f, cv::aruco::DICT_6X6_250)
  {
    finder = std::make_shared<CharucoGridBoardTargetFinder>(target);
  }

  virtual void SetUp() override
  {
    filename = std::string(TEST_SUPPORT_DIR) + "charuco_unobscured.jpg";

    // Expect all corners to be detected
    n_expected_ids = target.board->chessboardCorners.size();

    // Set up the homography check correspondence sampler
    auto grid_size = target.board->getChessboardSize();
    corr_sampler = std::make_shared<GridCorrespondenceSampler>(grid_size.height - 1, grid_size.width - 1, 1);
    homography_error_threshold = 1.0;
  }

  const CharucoGridTarget target;
};

class CharucoFinderPredefinedBoardTest : public TargetFinderTestFixture
{
public:
  CharucoFinderPredefinedBoardTest()
    : TargetFinderTestFixture()
    , target(cv::aruco::CharucoBoard::create(5, 7, 0.02f, 0.01f,
                                             cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250)))
  {
    finder = std::make_shared<CharucoGridBoardTargetFinder>(target);
  }

  virtual void SetUp() override
  {
    filename = std::string(TEST_SUPPORT_DIR) + "charuco_unobscured.jpg";

    // Expect all corners to be detected
    n_expected_ids = target.board->chessboardCorners.size();

    // Set up the homography check correspondence sampler
    auto grid_size = target.board->getChessboardSize();
    corr_sampler = std::make_shared<GridCorrespondenceSampler>(grid_size.height - 1, grid_size.width - 1, 1);
    homography_error_threshold = 1.0;
  }

  const CharucoGridTarget target;
};

class ObscuredCharucoFinderTest : public CharucoFinderTest
{
public:
  using CharucoFinderTest::CharucoFinderTest;

  virtual void SetUp() override final
  {
    filename = std::string(TEST_SUPPORT_DIR) + "charuco_obscured.jpg";

    // Expect the first seven corners not to be detected
    n_expected_ids = target.board->chessboardCorners.size() - 7;

    // Don't set up the homography check correspondence sampler because of the removed points
  }
};

class OneFeatureCharucoFinderTest : public CharucoFinderTest
{
public:
  using CharucoFinderTest::CharucoFinderTest;

  virtual void SetUp() override final
  {
    filename = std::string(TEST_SUPPORT_DIR) + "charuco_one_corner.jpg";

    // Expect to only see the first corner (ID = 0)
    n_expected_ids = 1;

    // Don't check homography because there are not enough points
  }
};

class ArucoFinderTest : public TargetFinderTestFixture
{
public:
  ArucoFinderTest() : TargetFinderTestFixture(), target(20, 20, 0.035f, 0.010f, cv::aruco::DICT_6X6_1000)
  {
    finder = std::make_shared<ArucoGridBoardTargetFinder>(target);
  }

  virtual void SetUp() override
  {
    filename = std::string(TEST_SUPPORT_DIR) + "aruco.jpg";

    // Expect to see the entire board
    n_expected_ids = static_cast<unsigned>(target.board->getGridSize().area());

    // Set up the homography check correspondence sampler
    // The stride of the correspondence sampler is 4, since there are 4 corners associated with each unique tag
    auto grid_size = target.board->getGridSize();
    corr_sampler = std::make_shared<GridCorrespondenceSampler>(grid_size.height, grid_size.width, 4);
    homography_error_threshold = 3.0;
  }

  ArucoGridTarget target;
};

class ArucoFinderPredefinedBoardTest : public TargetFinderTestFixture
{
public:
  ArucoFinderPredefinedBoardTest()
    : TargetFinderTestFixture()
    , target(cv::aruco::GridBoard::create(20, 20, 0.035f, 0.010f,
                                          cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000)))
  {
    finder = std::make_shared<ArucoGridBoardTargetFinder>(target);
  }

  virtual void SetUp() override
  {
    filename = std::string(TEST_SUPPORT_DIR) + "aruco.jpg";

    // Expect to see the entire board
    n_expected_ids = static_cast<unsigned>(target.board->getGridSize().area());

    // Set up the homography check correspondence sampler
    // The stride of the correspondence sampler is 4, since there are 4 corners associated with each unique tag
    auto grid_size = target.board->getGridSize();
    corr_sampler = std::make_shared<GridCorrespondenceSampler>(grid_size.height, grid_size.width, 4);
    homography_error_threshold = 3.0;
  }

  ArucoGridTarget target;
};

TEST_F(ModifiedCircleGridFinderTest, preservesOverlayInput) { this->verifyOverlayPreservesInput(); }

TEST_F(CharucoFinderTest, preservesOverlayInput) { this->verifyOverlayPreservesInput(); }

TEST(CharucoOverlayTest, drawsCornersWhenOnlyOneRowIsVisible)
{
  constexpr int rows = 6;
  constexpr int cols = 9;
  constexpr int tile = 80;
  constexpr int margin = 40;
  const CharucoGridTarget target(rows, cols, 0.02f, 0.014f, cv::aruco::DICT_6X6_250);
  const CharucoGridBoardTargetFinder finder(target);

  cv::Mat board_image;
  target.board->draw({ cols * tile + 2 * margin, rows * tile + 2 * margin }, board_image, margin);
  cv::Mat partial_image(board_image.size(), board_image.type(), cv::Scalar(255));
  const cv::Range visible_rows(margin + 2 * tile, margin + 4 * tile);
  board_image.rowRange(visible_rows).copyTo(partial_image.rowRange(visible_rows));
  cv::Mat image;
  cv::cvtColor(partial_image, image, cv::COLOR_GRAY2BGR);

  const auto features = finder.findTargetFeatures(image);
  ASSERT_EQ(features.size(), static_cast<std::size_t>(cols - 1));
  const auto target_y = target.points.at(features.begin()->first).y();
  for (const auto& feature : features)
    EXPECT_EQ(target.points.at(feature.first).y(), target_y);

  cv::Mat overlay;
  ASSERT_NO_THROW(overlay = finder.drawTargetFeatures(image, features));
  EXPECT_GT(cv::norm(overlay, image, cv::NORM_INF), 0.0);
}

TEST_F(ArucoFinderTest, preservesOverlayInput) { this->verifyOverlayPreservesInput(); }

TEST_F(ModifiedCircleGridFinderTest, findTargetFeatures) { this->runTest(); }

TEST_F(CharucoFinderTest, findTargetFeatures) { this->runTest(); }

TEST(CircleDetectorMaskTest, filtersKeypointsByImageMask)
{
  const cv::Mat image = cv::imread(std::string(TEST_SUPPORT_DIR) + "modified_circle_grid_10x10_0.025.png");
  ASSERT_FALSE(image.empty());
  CircleDetector detector(CircleDetectorParams{});
  std::vector<cv::KeyPoint> keypoints;
  detector.detect(image, keypoints);
  ASSERT_FALSE(keypoints.empty());

  const cv::Mat mask = cv::Mat::zeros(image.size(), CV_8UC1);
  detector.detect(image, keypoints, mask);
  EXPECT_TRUE(keypoints.empty());
}

TEST_F(CharucoFinderPredefinedBoardTest, findTargetFeatures) { this->runTest(); }

TEST_F(ObscuredCharucoFinderTest, findTargetFeatures) { this->runTest(); }

TEST_F(OneFeatureCharucoFinderTest, findTargetFeatures) { this->runTest(); }

TEST_F(ArucoFinderTest, findTargetFeatures) { this->runTest(); }

TEST_F(ArucoFinderTest, usesSingleDetectionForCorrespondences)
{
  class SingleDetectionFinder final : public ArucoGridBoardTargetFinder
  {
  public:
    using ArucoGridBoardTargetFinder::ArucoGridBoardTargetFinder;

    TargetFeatures2D findTargetFeatures(const cv::Mat& image) const override
    {
      if (++calls == 1)
        return ArucoGridBoardTargetFinder::findTargetFeatures(image);
      return {};
    }

    mutable unsigned calls = 0;
  };

  const cv::Mat image = cv::imread(filename);
  ASSERT_FALSE(image.empty());
  const SingleDetectionFinder single_detection(target);

  const auto correspondences = single_detection.findCorrespondences(image);

  EXPECT_EQ(single_detection.calls, 1u);
  EXPECT_EQ(correspondences.size(), 4 * n_expected_ids);
}

TEST_F(ArucoFinderPredefinedBoardTest, findTargetFeatures) { this->runTest(); }

TEST(ArucoBoardMembershipTest, ignoresMarkersFromOutsideConfiguredBoard)
{
  constexpr int foreign_id = 100;
  const ArucoGridTarget target(2, 2, 0.04f, 0.01f);
  const ArucoGridBoardTargetFinder finder(target);
  cv::Mat scene(260, 410, CV_8UC1, cv::Scalar(255));
  cv::Mat board_image;
  target.board->draw({ 240, 240 }, board_image, 20);
  board_image.copyTo(scene(cv::Rect(10, 10, 240, 240)));
  cv::Mat foreign_marker;
  cv::aruco::drawMarker(target.board->dictionary, foreign_id, 100, foreign_marker);
  foreign_marker.copyTo(scene(cv::Rect(275, 80, 100, 100)));

  const auto features = finder.findTargetFeatures(scene);
  ASSERT_EQ(features.size(), target.points.size());
  EXPECT_EQ(features.count(foreign_id), 0);
  EXPECT_EQ(target.createCorrespondences(features).size(), 4 * target.points.size());
}

TEST(MarkerBoardConfigurationTest, acceptsSupportedDictionaryBounds)
{
  for (const auto id : { cv::aruco::DICT_4X4_50, cv::aruco::DICT_APRILTAG_36h11 })
  {
    const auto dictionary = cv::aruco::getPredefinedDictionary(id);
    const ArucoGridTarget aruco(2, 3, 0.035f, 0.010f, id);
    const CharucoGridTarget charuco(5, 7, 0.036195f, 0.018256f, id);
    EXPECT_EQ(aruco.board->dictionary->bytesList.rows, dictionary->bytesList.rows);
    EXPECT_EQ(charuco.board->dictionary->bytesList.rows, dictionary->bytesList.rows);
  }
}

TEST(MarkerBoardConfigurationTest, rejectsUnknownDictionaryIds)
{
  for (const int id : { -1, static_cast<int>(cv::aruco::DICT_APRILTAG_36h11) + 1 })
  {
    EXPECT_THROW(ArucoGridTarget(2, 3, 0.035f, 0.010f, id), ICalException);
    EXPECT_THROW(CharucoGridTarget(5, 7, 0.036195f, 0.018256f, id), ICalException);
  }
}

TEST(MarkerBoardConfigurationTest, acceptsBoardsAtDictionaryCapacity)
{
  const ArucoGridTarget aruco(5, 10, 0.035f, 0.010f, cv::aruco::DICT_4X4_50);
  EXPECT_EQ(aruco.board->ids.size(), 50u);

  const CharucoGridTarget charuco(25, 47, 0.036195f, 0.018256f, cv::aruco::DICT_APRILTAG_36h11);
  EXPECT_EQ(charuco.board->ids.size(), 587u);
}

TEST(MarkerBoardConfigurationTest, rejectsBoardsOverDictionaryCapacity)
{
  constexpr auto dictionary = cv::aruco::DICT_4X4_50;
  EXPECT_THROW(ArucoGridTarget(5, 11, 0.035f, 0.010f, dictionary), ICalException);
  EXPECT_THROW(CharucoGridTarget(10, 11, 0.036195f, 0.018256f, dictionary), ICalException);
}

TEST(CircleGridConfigurationTest, requiresFinitePositiveSpacing)
{
  EXPECT_NO_THROW(ModifiedCircleGridTarget(10, 10, 0.025));
  for (const double spacing :
       { 0.0, -0.025, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity() })
    EXPECT_THROW(ModifiedCircleGridTarget(10, 10, spacing), ICalException);
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
