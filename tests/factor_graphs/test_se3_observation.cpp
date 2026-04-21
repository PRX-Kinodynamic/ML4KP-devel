#define BOOST_AUTO_TEST_MAIN se3_observation_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/defs.hpp"
#include "prx/factor_graphs/factors/se3_observation.hpp"

#include <gtsam/base/numericalDerivative.h>

const double tolerance{ 1e-5 };
BOOST_AUTO_TEST_CASE(SE3_observation_factor)
{
  using EndcapFactor = prx::fg::SE3_observation_factor_t;

  using SE3 = EndcapFactor::SE3;
  using Translation = EndcapFactor::Translation;

  // quat, pos
  const SE3 q{ 0.0, 1.0, 0.0, 0.0,  // quat
               3.0, 2.0, 5.0 };     // pos
  const Translation offset{ 0, 0, 3.25 / 2.0 };

  // Check jacobians
  Eigen::MatrixXd actualHq, expectedHq;
  actualHq = Eigen::Matrix<double, 3, 6>::Zero();
  expectedHq = Eigen::Matrix<double, 3, 6>::Zero();

  EndcapFactor factor(0, offset, Translation::Zero() * 0.1, nullptr);

  std::function<Translation(const SE3&)> vd_proxy = [&](const SE3& x_) { return factor.evaluateError(x_); };

  factor.evaluateError(q, actualHq);
  expectedHq = gtsam::numericalDerivative11(vd_proxy, q);

  BOOST_REQUIRE(expectedHq.isApprox(actualHq, tolerance));
}