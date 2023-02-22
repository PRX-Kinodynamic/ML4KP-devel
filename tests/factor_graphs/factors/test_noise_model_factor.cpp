#define BOOST_AUTO_TEST_MAIN noise_model_factor_test
#include <cmath>
#include <string>

#include <boost/test/unit_test.hpp>

#include "prx/factor_graphs/factors/noise_model_factor.hpp"

const double tolerance{ 0.001 };
struct test_noise_model_1factor : public prx::fg::noise_model_1factor_t<2>
{
  test_noise_model_1factor() : noise_model_1factor_t(gtsam::Key(), gtsam::noiseModel::Isotropic::Sigma(2, 1e-3))
  {
  }

  virtual X0 compute_error(const X0& x0) const override
  {
    return X0(std::pow(x0[0], 2), std::pow(x0[1], 2));
  }
};

BOOST_AUTO_TEST_CASE(noise_model_1factor_test)
{
  test_noise_model_1factor factor;
  Eigen::Vector2d x0(1, 2);
  Eigen::MatrixXd h{ Eigen::MatrixXd::Zero(2, 2) };
  Eigen::Vector2d error = factor.evaluateError(x0, h);

  Eigen::Vector2d expected_error(1, 4);
  Eigen::MatrixXd expected_H(2, 2);
  expected_H << 2, 0, 0, 4;  // [2x,0;0,2y]
  BOOST_CHECK_SMALL((error - expected_error).norm(), tolerance);
  BOOST_CHECK_SMALL((h - expected_H).norm(), tolerance);
}

struct test_noise_model_2factor : public prx::fg::noise_model_2factor_t<2, 2>
{
  test_noise_model_2factor()
    : noise_model_2factor_t(gtsam::Key(), gtsam::Key(), gtsam::noiseModel::Isotropic::Sigma(2, 1e-3))
  {
  }

  virtual X0 compute_error(const X0& x0, const X1& x1) const override
  {
    X1 val{ Eigen::sin(x1.array()) };
    return x0 - val;
  }
};

BOOST_AUTO_TEST_CASE(noise_model_2factor_test)
{
  test_noise_model_2factor factor;
  Eigen::Vector2d x0(0, 1);      // sin(x=(0,pi/2))
  Eigen::Vector2d x1(0.1, 1.5);  // (0,pi/2) + epsilon
  Eigen::MatrixXd h0{ Eigen::MatrixXd::Zero(2, 2) };
  Eigen::MatrixXd h1{ Eigen::MatrixXd::Zero(2, 2) };
  Eigen::Vector2d error = factor.evaluateError(x0, x1, h0, h1);

  Eigen::Vector2d expected_error(-0.09983341665, 0.002505013396);  // [-sin(0.1),1-sin(1.5)]
  Eigen::MatrixXd expected_H0(2, 2);                               // [1,0;0,1]
  Eigen::MatrixXd expected_H1(2, 2);                               // [-cos(0.1),0;0,-cos(1.5)]
  expected_H0 << 1, 0, 0, 1;
  expected_H1 << -std::cos(0.1), 0, 0, -std::cos(1.5);
  BOOST_CHECK_SMALL((error - expected_error).norm(), tolerance);
  BOOST_CHECK_SMALL((h0 - expected_H0).norm(), tolerance);
  BOOST_CHECK_SMALL((h1 - expected_H1).norm(), tolerance);
}

struct test_noise_model_3factor : public prx::fg::noise_model_3factor_t<1, 2, 2>
{
  test_noise_model_3factor()
    : noise_model_3factor_t(gtsam::Key(), gtsam::Key(), gtsam::Key(), gtsam::noiseModel::Isotropic::Sigma(2, 1e-3))
  {
  }

  virtual X0 compute_error(const X0& x0, const X1& x1, const X2& x2) const override
  {
    return x0 - X0((x1 - x2).norm());
  }
};

BOOST_AUTO_TEST_CASE(noise_model_3factor_test)
{
  test_noise_model_3factor factor;
  using X0 = Eigen::Vector<double, 1>;
  using X1 = Eigen::Vector<double, 2>;
  using X2 = Eigen::Vector<double, 2>;
  X0 x0(1);         // distance=1
  X1 x1(1, 2);      //
  X2 x2(1.9, 2.1);  //
  Eigen::MatrixXd h0{ Eigen::MatrixXd::Zero(1, 1) };
  Eigen::MatrixXd h1{ Eigen::MatrixXd::Zero(1, 1) };
  Eigen::MatrixXd h2{ Eigen::MatrixXd::Zero(1, 1) };
  X0 error = factor.evaluateError(x0, x1, x2, h0, h1, h2);

  X0 expected_error(1 - 0.9055385138);  // [1-distance(x1,x2)]
  Eigen::MatrixXd expected_H0(1, 1);    // [1]
  // The error is: X0 - f(X1-X2)l f=norm()
  // d norm/dx = x/sqrt(x^2+y^2)
  // d norm/dy = y/sqrt(x^2+y^2)
  Eigen::MatrixXd expected_H1(1, 2);  // [d norm/dx, d norm/dy]
  Eigen::MatrixXd expected_H2(1, 2);  // [-d norm/dx, -d norm/dy]
  expected_H0 << 1;
  expected_H1 << 0.9938837347, 0.1104315261;    //
  expected_H2 << -0.9938837347, -0.1104315261;  //
  BOOST_CHECK_SMALL((error - expected_error).norm(), tolerance);
  BOOST_CHECK_SMALL((h0 - expected_H0).norm(), tolerance);
  BOOST_CHECK_SMALL((h1 - expected_H1).norm(), tolerance);
  BOOST_CHECK_SMALL((h2 - expected_H2).norm(), tolerance);
}
// TODO: Add tests for test_noise_model_4factor && test_noise_model_5factor