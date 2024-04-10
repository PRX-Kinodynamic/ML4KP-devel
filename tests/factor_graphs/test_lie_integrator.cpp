#define BOOST_AUTO_TEST_MAIN lie_integrator_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/defs.hpp"
#include "prx/factor_graphs/lie_groups/lie_integrator.hpp"
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/SOn.h>
namespace mock
{
// using X = Eigen::Vector<double, 3>;
using Rdot = Eigen::Vector<double, 1>;
struct SO2 : public gtsam::SO<2>
{
  using Scalar = double;
  // static constexpr Eigen::Index dimension = 1;
  static constexpr Eigen::Index RowsAtCompileTime = 1;
  SO2(double theta)
    : gtsam::SO<2>(
          (Eigen::Matrix2d() << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta)).finished())
  {
  }

  SO2(const gtsam::SO<2>& so2) : gtsam::SO<2>(so2)
  {
  }

  SO2(const SO2& so2) : gtsam::SO<2>(so2)
  {
  }

  // SO2 operator+(const Rdot& other) const
  // {
  //   const SO2 manif_other(SO2::expmap(other));

  //   return (*this) * manif_other;
  // }

  // Rdot operator-(const SO2& other) const
  // {
  //   const double LHS(SO2::Logmap(*this));
  //   const double RHS(SO2::Logmap(other));
  //   return Rdot(LHS - RHS);
  // }

  // template <typename Rdot>
  static SO2 Expmap(const Rdot& wz, gtsam::OptionalJacobian<1, 1> H = boost::none)
  {
    const SO2 so2(wz[0]);
    return so2;
  }

  static gtsam::SO<2>::TangentVector Logmap(const SO2& so2)
  {
    const Eigen::Matrix<double, 2, 2> mat{ so2.matrix() };
    const double r21{ mat(1, 0) };
    const double r11{ mat(0, 0) };
    return gtsam::SO<2>::TangentVector(std::atan2(r21, r11));
  }

  SO2 compose(const SO2& g, ChartJacobian H1, ChartJacobian H2 = boost::none) const
  {
    SO2 res{ this->gtsam::SO<2>::compose(g, H1, H2) };
    return res;
  }

  friend std::ostream& operator<<(std::ostream& os, const SO2& so2)
  {
    os << so2.matrix();
    return os;
  }
};

}  // namespace mock
namespace gtsam
{
template <>
struct traits<mock::SO2> : public internal::LieGroup<mock::SO2>
{
  static constexpr Eigen::Index dimension = 1;
  static constexpr int GetDimension(const mock::SO2&)
  {
    return dimension;
  }
};
}  // namespace gtsam
BOOST_AUTO_TEST_CASE(propagete_without_derivatives_test)
{
  using Integrator = prx::fg::lie_integrator_t<mock::SO2, mock::Rdot>;
  const double dt{ 0.1 };
  const double angle0{ 0 };
  const double angle_dot0{ 1 };
  const mock::SO2 x0{ angle0 };
  const mock::Rdot xdot0(angle_dot0);

  const mock::SO2 x1{ Integrator::integrate(x0, xdot0, dt) };

  const double angle_1{ angle0 + angle_dot0 * dt };
  const mock::SO2 x1_p{ angle_1 };

  const Eigen::Matrix2d expected{ x1_p.matrix() };
  const Eigen::Matrix2d result{ x1.matrix() };
  BOOST_REQUIRE_MESSAGE(expected.isApprox(result), EXPECTED_GOT(expected, result));
}

BOOST_AUTO_TEST_CASE(propagete_without_derivatives_multiple_steps_test)
{
  using Integrator = prx::fg::lie_integrator_t<mock::SO2, mock::Rdot>;
  const double dt{ 0.1 };
  const double angle0{ 0 };
  const mock::SO2 x0{ angle0 };
  const mock::Rdot xdot0(1);
  const mock::Rdot xdot1(2);
  const mock::Rdot xdot2(3);

  const mock::SO2 x1{ Integrator::integrate(x0, xdot0, dt) };
  const mock::SO2 x2{ Integrator::integrate(x1, xdot1, dt) };
  const mock::SO2 x3{ Integrator::integrate(x2, xdot2, dt) };

  const double angle1{ angle0 + xdot0[0] * dt };
  const double angle2{ angle1 + xdot1[0] * dt };
  const double angle3{ angle2 + xdot2[0] * dt };
  const mock::SO2 x1_p{ angle1 };
  const mock::SO2 x2_p{ angle2 };
  const mock::SO2 x3_p{ angle3 };

  const Eigen::Matrix2d expected{ x3_p.matrix() };
  const Eigen::Matrix2d result{ x3.matrix() };
  BOOST_REQUIRE_MESSAGE(expected.isApprox(result), EXPECTED_GOT(expected, result));
}

BOOST_AUTO_TEST_CASE(optimizer_find_x1_test)
{
  using IntegrationFactor = prx::fg::lie_integration_factor_t<mock::SO2, mock::Rdot>;

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;
  const double dt{ 0.1 };
  const double w{ 1 };

  mock::SO2 x0(0);
  mock::SO2 x1(0);
  mock::Rdot xdot0(w);

  const gtsam::Key key_x0{ gtsam::Symbol('x', 0) };
  const gtsam::Key key_x1{ gtsam::Symbol('x', 1) };
  const gtsam::Key key_xdot{ gtsam::Symbol('D', 1) };
  graph.emplace_shared<IntegrationFactor>(key_x1, key_x0, key_xdot, nullptr, dt);

  initial_values.insert(key_x0, x0);
  initial_values.insert(key_x1, x1);
  initial_values.insert(key_xdot, xdot0);
  graph.addPrior(key_x0, x0);
  graph.addPrior(key_xdot, xdot0);
  gtsam::Values values = gtsam::GaussNewtonOptimizer(graph, initial_values).optimize();
  // values.print();

  const mock::SO2 x1_p{ w * dt };  // 0 + w*dt

  const Eigen::Matrix2d expected{ x1_p.matrix() };
  const Eigen::Matrix2d result{ values.at<mock::SO2>(key_x1).matrix() };
  BOOST_REQUIRE_MESSAGE(expected.isApprox(result), EXPECTED_GOT(expected, result));
}

BOOST_AUTO_TEST_CASE(optimizer_find_x0_test)
{
  // Test that the FG is able to find x1 <- x0 + xdot * dt given that x0=x1 with priors on x1 and xdot
  // This is, we know *perfectly* x1 and xdot but want to find x0
  using IntegrationFactor = prx::fg::lie_integration_factor_t<mock::SO2, mock::Rdot>;

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;
  const double dt{ 0.1 };
  const double w{ 1 };

  mock::SO2 x0(w * dt);
  mock::SO2 x1(w * dt);
  mock::Rdot xdot0(w);

  const gtsam::Key key_x0{ gtsam::Symbol('x', 0) };
  const gtsam::Key key_x1{ gtsam::Symbol('x', 1) };
  const gtsam::Key key_xdot{ gtsam::Symbol('D', 1) };
  graph.emplace_shared<IntegrationFactor>(key_x1, key_x0, key_xdot, nullptr, dt);

  initial_values.insert(key_x0, x0);
  initial_values.insert(key_x1, x1);
  initial_values.insert(key_xdot, xdot0);
  graph.addPrior(key_x1, x1);
  graph.addPrior(key_xdot, xdot0);
  gtsam::Values values = gtsam::GaussNewtonOptimizer(graph, initial_values).optimize();
  // values.print();

  const mock::SO2 x0_p{ 0 };  // 0 + w*dt

  const Eigen::Matrix2d expected{ x0_p.matrix() };
  const Eigen::Matrix2d result{ values.at<mock::SO2>(key_x0).matrix() };
  BOOST_REQUIRE_MESSAGE(expected.isApprox(result), EXPECTED_GOT(expected, result));
}

BOOST_AUTO_TEST_CASE(optimizer_find_xdot_test)
{
  // Test that the FG is able to find x1 <- x0 + xdot * dt given that x0 and x1 are known *perfectly*
  // but want to find xdot
  using IntegrationFactor = prx::fg::lie_integration_factor_t<mock::SO2, mock::Rdot>;

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;
  const double dt{ 0.1 };
  const double w{ 1 };

  mock::SO2 x0(0);
  mock::SO2 x1(w * dt);
  mock::Rdot xdot0(0.5);

  const gtsam::Key key_x0{ gtsam::Symbol('x', 0) };
  const gtsam::Key key_x1{ gtsam::Symbol('x', 1) };
  const gtsam::Key key_xdot{ gtsam::Symbol('D', 1) };
  graph.emplace_shared<IntegrationFactor>(key_x1, key_x0, key_xdot, nullptr, dt);

  initial_values.insert(key_x0, x0);
  initial_values.insert(key_x1, x1);
  initial_values.insert(key_xdot, xdot0);
  graph.addPrior(key_x0, x0);
  graph.addPrior(key_x1, x1);
  gtsam::GaussNewtonParams params{};
  params.setRelativeErrorTol(1e-10);  // Need to set this lower than default
  params.setAbsoluteErrorTol(1e-10);
  gtsam::Values values = gtsam::GaussNewtonOptimizer(graph, initial_values, params).optimize();
  // values.print();

  const mock::Rdot expected{ w };  // 0 + w*dt

  const mock::Rdot result{ values.at<mock::Rdot>(key_xdot) };
  BOOST_REQUIRE_MESSAGE(expected.isApprox(result, 1e-3), EXPECTED_GOT(expected, result));
}