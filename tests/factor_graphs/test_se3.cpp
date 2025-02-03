#define BOOST_AUTO_TEST_MAIN se3_test
#include <string>
#include <boost/test/unit_test.hpp>
#include <gtsam/base/numericalDerivative.h>
#include "prx/utilities/defs.hpp"
#include "prx/factor_graphs/lie_groups/se3.hpp"

using SE3 = prx::fg::se3_t;
using Quaternion = Eigen::Quaterniond;
using Position = Eigen::Vector<double, 3>;
using Twist = Eigen::Vector<double, 6>;

BOOST_AUTO_TEST_CASE(constructor_test)
{
  const SE3 se3(Quaternion::Identity(), Position::Zero());

  const Quaternion q_expected{ Quaternion::Identity() };
  const Position p_expected{ Position::Zero() };
  BOOST_REQUIRE_MESSAGE(q_expected.isApprox(se3.quaternion()), EXPECTED_GOT(q_expected, se3.quaternion()));
  BOOST_REQUIRE_MESSAGE(p_expected.isApprox(se3.position()), EXPECTED_GOT(p_expected, se3.position()));
}

BOOST_AUTO_TEST_CASE(composition_test)
{
  using RotationMatrix = Eigen::Matrix<double, 3, 3>;
  Position p0(0, -2, 0);
  Position p1(-1, 1, 0);
  Position p01(0, -3, -1);
  RotationMatrix R0{}, R1{}, R01{};
  R0 << 0, 0, 1, 0, -1, 0, 1, 0, 0;
  R1 << -1, 0, 0, 0, 0, 1, 0, 1, 0;
  R01 << 0, 1, 0, 0, 0, -1, -1, 0, 0;

  const SE3 se3_0(R0, p0);
  const SE3 se3_1(R1, p1);
  const SE3 se3_01{ se3_0 * se3_1 };

  BOOST_REQUIRE_MESSAGE(R01.isApprox(se3_01.rotation_matrix()), EXPECTED_GOT(R01, se3_01.rotation_matrix()));
  BOOST_REQUIRE_MESSAGE(p01.isApprox(se3_01.position()), EXPECTED_GOT(p01, se3_01.position()));
}

// Eqs. 2,3,4  Micro-lie theory: exp((t+s)τ∧)=exp(tτ∧)exp(sτ∧)
BOOST_AUTO_TEST_CASE(expmap_compose_test)
{
  using Traits = gtsam::traits<SE3>;
  const SE3 Id{ SE3() };

  for (int i = 0; i < 100; ++i)
  {
    const SE3 xrand{ SE3::Expmap(Eigen::Vector<double, 6>::Random()) };    // not the best random...
    const SE3 xrand_x{ SE3::Expmap(Eigen::Vector<double, 6>::Random()) };  // not the best random...
    const SE3 xrand_y{ SE3::Expmap(Eigen::Vector<double, 6>::Random()) };  // not the best random...
    const SE3 xrand_z{ SE3::Expmap(Eigen::Vector<double, 6>::Random()) };  // not the best random...

    const SE3 x_eq2_0{ Traits::Compose(xrand, Id) };
    const SE3 x_eq2_1{ Traits::Compose(Id, xrand) };

    const SE3 x_eq3_0{ Traits::Compose(xrand, xrand.inverse()) };
    const SE3 x_eq3_1{ Traits::Compose(xrand.inverse(), xrand) };

    const SE3 x_eq4_0{ Traits::Compose(Traits::Compose(xrand_x, xrand_y), xrand_z) };
    const SE3 x_eq4_1{ Traits::Compose(xrand_x, Traits::Compose(xrand_y, xrand_z)) };

    BOOST_REQUIRE(x_eq2_0.equals(x_eq2_1));
    BOOST_REQUIRE_MESSAGE(x_eq3_0.equals(Id), EXPECTED_GOT(Id, x_eq3_0));
    BOOST_REQUIRE_MESSAGE(x_eq3_1.equals(Id), EXPECTED_GOT(Id, x_eq3_1));
    BOOST_REQUIRE(x_eq4_0.equals(x_eq4_1));
  }
}

// Eq. 17 Micro-lie theory: exp((t+s)τ∧)=exp(tτ∧)exp(sτ∧)
BOOST_AUTO_TEST_CASE(expmap_eq17_test)
{
  for (int i = 0; i < 100; ++i)
  {
    const double t{ prx::uniform_random(0.0, 0.5) };
    const double s{ prx::uniform_random(0.0, 0.5) };
    const Eigen::Vector<double, 6> tau_rand{ Eigen::Vector<double, 6>::Random() };

    const SE3 x0{ SE3::Expmap((t + s) * tau_rand) };
    const SE3 x1{ SE3::Expmap(t * tau_rand) * SE3::Expmap(s * tau_rand) };
    BOOST_REQUIRE(x0.equals(x1));
  }
}

// Eq. 19 Micro-lie theory
BOOST_AUTO_TEST_CASE(expmap_eq19_test)
{
  for (int i = 0; i < 100; ++i)
  {
    const Eigen::Vector<double, 6> tau_rand{ Eigen::Vector<double, 6>::Random() };
    const SE3 x0{ SE3::Expmap(-tau_rand) };
    const SE3 x1{ SE3::Expmap(tau_rand).inverse() };
    BOOST_REQUIRE(x0.equals(x1));
  }
}

// Logmap
BOOST_AUTO_TEST_CASE(logmap_test)
{
  for (int i = 0; i < 100; ++i)
  {
    const Eigen::Vector<double, 6> tau_rand{ Eigen::Vector<double, 6>::Random() };
    const SE3 x0{ SE3::Expmap(tau_rand) };
    const Eigen::Vector<double, 6> tau_res{ SE3::Logmap(x0) };

    const Eigen::Vector<double, 6> diff{ tau_rand - tau_res };
    BOOST_REQUIRE(diff.isZero(1e-8));
  }
}

// TEST(TestQuadrotorFactors, testQuadrotorIntegrate)
BOOST_AUTO_TEST_CASE(test_compose)
{
  // using Integrator = prx::fg::lie_integrator_t<SE3, Twist, double>;
  // using Integrator = prx::fg::lie_integration_factor_t<State, StateDot, double>;
  // using QuadrotorIntegrator = prx_models::quadrotor_integrator_factor_t;

  using State = SE3;
  const SE3 x0{ SE3(0.8775826, 0.0, 0.0, 0.4794255, 1, 2, 3) };
  const SE3 x1{ SE3(0.0, 0.8775826, 0.4794255, 0.0, 0, 3, 2) };
  // using State = gtsam::Pose3;
  // const gtsam::Pose3 x0{ gtsam::Rot3(0.8775826, 0.0, 0.0, 0.4794255), Eigen::Vector3d(1, 2, 3) };
  // const gtsam::Pose3 x1{ gtsam::Rot3(0.0, 0.8775826, 0.4794255, 0.0), Eigen::Vector3d(0, 3, 2) };

  // Check jacobians
  Eigen::MatrixXd actualHx0, expectedHx0;
  Eigen::MatrixXd actualHx1, expectedHx1;

  std::function<State(const State&, const State&)> err_proxy =  // no-lint
      [](const State& x0_, const State& x1_) { return x0_.compose(x1_); };
  // numericalDerivative21
  x0.compose(x1, actualHx0, actualHx1);
  expectedHx0 = gtsam::numericalDerivative21(err_proxy, x0, x1);
  expectedHx1 = gtsam::numericalDerivative22(err_proxy, x0, x1);

  PRX_DBG_VARS(expectedHx0);
  PRX_DBG_VARS(actualHx0);

  PRX_DBG_VARS(expectedHx1);
  PRX_DBG_VARS(actualHx1);

  const double tolerance{ 1e-4 };
  const Eigen::MatrixXd diff0{ expectedHx0 - actualHx0 };
  const Eigen::MatrixXd diff1{ expectedHx1 - actualHx1 };

  const bool expectedHx0_isApprox_actualHx0{ diff0.isZero(tolerance) };
  const bool expectedHx1_isApprox_actualHx1{ diff1.isZero(tolerance) };

  PRX_DBG_VARS(diff0);
  PRX_DBG_VARS(diff1);

  PRX_DBG_VARS(expectedHx0_isApprox_actualHx0);
  PRX_DBG_VARS(expectedHx1_isApprox_actualHx1);

  BOOST_REQUIRE(expectedHx0_isApprox_actualHx0);
  BOOST_REQUIRE(expectedHx1_isApprox_actualHx1);
}
