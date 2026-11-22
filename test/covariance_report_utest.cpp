#include <calibration_workbench/optimizations/covariance_analysis.h>

#include <cmath>
#include <gtest/gtest.h>
#include <limits>

using namespace calibration_workbench;

TEST(CovarianceReport, KeepsZeroStandardDeviationAfterReordering)
{
  const std::vector<std::string> names{ "x", "y" };
  for (const Eigen::Index fixed : { 0, 1 })
  {
    SCOPED_TRACE(fixed);
    const Eigen::Index free = 1 - fixed;
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    covariance(free, free) = 4.0;

    const auto report = computeCovarianceResults(covariance, names);
    EXPECT_DOUBLE_EQ(report.correlation_matrix(fixed, fixed), 0.0);
    EXPECT_DOUBLE_EQ(report.correlation_matrix(free, free), 2.0);
    EXPECT_DOUBLE_EQ(report.correlation_matrix(0, 1), 0.0);
    EXPECT_DOUBLE_EQ(report.correlation_matrix(1, 0), 0.0);

    ASSERT_EQ(report.standard_deviations.size(), names.size());
    EXPECT_EQ(report.standard_deviations.at(fixed).names.first, names.at(fixed));
    EXPECT_DOUBLE_EQ(report.standard_deviations.at(fixed).value, 0.0);
    EXPECT_DOUBLE_EQ(report.standard_deviations.at(free).value, 2.0);
  }
}

TEST(CovarianceReport, PreservesStrongCorrelationsAcrossUnitScales)
{
  for (const double rho : { 0.6, -0.6 })
  {
    SCOPED_TRACE(rho);
    for (const auto& sigma : { Eigen::Vector2d(1.0, 2.0), Eigen::Vector2d(1e-20, 2e-20), Eigen::Vector2d(1.0, 2e-20),
                               Eigen::Vector2d(1e-20, 2.0) })
    {
      SCOPED_TRACE(sigma.transpose());
      Eigen::Matrix2d covariance;
      const double cross_covariance = rho * sigma[0] * sigma[1];
      covariance << sigma[0] * sigma[0], cross_covariance, cross_covariance, sigma[1] * sigma[1];

      const auto report = computeCovarianceResults(covariance, { "x", "y" });
      ASSERT_EQ(report.standard_deviations.size(), 2u);
      EXPECT_DOUBLE_EQ(report.standard_deviations[0].value, sigma[0]);
      EXPECT_DOUBLE_EQ(report.standard_deviations[1].value, sigma[1]);
      EXPECT_NEAR(report.correlation_matrix(0, 1), rho, 1e-15);
      EXPECT_DOUBLE_EQ(report.correlation_matrix(0, 1), report.correlation_matrix(1, 0));

      const auto strong_correlations = report.getCorrelationCoeffOutsideThreshold(0.5);
      ASSERT_EQ(strong_correlations.size(), 1u);
      EXPECT_EQ(strong_correlations.front().names, std::make_pair(std::string("x"), std::string("y")));
      EXPECT_NEAR(strong_correlations.front().value, rho, 1e-15);
    }
  }
}

TEST(CovarianceReport, NormalizesSubnormalCovariances)
{
  const double variance = std::numeric_limits<double>::denorm_min();
  ASSERT_GT(variance, 0.0);
  Eigen::Matrix2d covariance;
  covariance << variance, variance, variance, 2.0 * variance;

  const auto result = computeCorrelationsFromCovariance(covariance);
  EXPECT_DOUBLE_EQ(result(0, 0), std::sqrt(variance));
  EXPECT_DOUBLE_EQ(result(1, 1), std::sqrt(2.0 * variance));
  EXPECT_NEAR(result(0, 1), 1.0 / std::sqrt(2.0), 1e-15);
  EXPECT_DOUBLE_EQ(result(0, 1), result(1, 0));
}
