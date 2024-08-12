#define BOOST_AUTO_TEST_MAIN lie_integrator_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/defs.hpp"
#include "prx/factor_graphs/factors/euler_integration_factor.hpp"

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/SOn.h>
namespace mock
{
using State = Eigen::Vector<double, 2>;
using StateDot = Eigen::Vector<double, 2>;

// Considering a non-actuated pendulum
StateDot pendulum_dyn(const State& x)
{
  const double m{ 1 };
  const double g{ -9.81 };
  const double l{ 1 };
  const double thetad{ x[1] };
  const double thetadd{ (m * g * l * std::sin(x[0]) - x[1]) / m * l * l };
  return StateDot(thetad, thetadd);
}

}  // namespace mock

BOOST_AUTO_TEST_CASE(propagate_without_derivatives_test)
{
  using Integrator = prx::fg::euler_integration_factor_t<mock::State, mock::StateDot>;
  const double dt{ 0.1 };
  const mock::State x0(0.1, 0);
  mock::StateDot xdot(0.0, 0.0);

  mock::State result{ x0 };
  for (int i = 0; i < 10000; ++i)
  {
    xdot = mock::pendulum_dyn(result);

    result = Integrator::integrate(result, xdot, dt);
  }

  const mock::State expected{ mock::State::Zero() };
  const double epsilon{ 0.01 };
  BOOST_REQUIRE_MESSAGE(result[0] < epsilon, EXPECTED_GOT(expected[0], result[0]));
  BOOST_REQUIRE_MESSAGE(result[1] < epsilon, EXPECTED_GOT(expected[1], result[1]));
}

BOOST_AUTO_TEST_CASE(optimizer_find_x0_test)
{
  using Integrator = prx::fg::euler_integration_factor_t<mock::State, mock::StateDot>;

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;
  const double dt{ 0.1 };
  const mock::State x0(0.1, 0);
  const mock::State x0_noise(0.0, 0);
  const mock::StateDot xdot(0.0, 0.0);
  const mock::State x1{ Integrator::integrate(x0, xdot, dt) };

  const gtsam::Key key_x0{ gtsam::Symbol('x', 0) };
  const gtsam::Key key_x1{ gtsam::Symbol('x', 1) };
  const gtsam::Key key_xdot{ gtsam::Symbol('D', 1) };
  graph.emplace_shared<Integrator>(key_x1, key_x0, key_xdot, nullptr, dt);

  initial_values.insert(key_x0, x0_noise);
  initial_values.insert(key_x1, x1);
  initial_values.insert(key_xdot, xdot);
  graph.addPrior(key_xdot, xdot);
  graph.addPrior(key_x1, x1);
  gtsam::GaussNewtonParams params{};
  params.setRelativeErrorTol(1e-10);  // Need to set this lower than default
  params.setAbsoluteErrorTol(1e-10);
  gtsam::Values values = gtsam::GaussNewtonOptimizer(graph, initial_values, params).optimize();
  // values.print();

  const mock::State expected{ x0 };  // 0 + w*dt

  const mock::State result{ values.at<mock::State>(key_x0) };
  BOOST_REQUIRE_MESSAGE(expected.isApprox(result, 1e-3), EXPECTED_GOT(expected, result));
}

BOOST_AUTO_TEST_CASE(optimizer_find_x1_test)
{
  using Integrator = prx::fg::euler_integration_factor_t<mock::State, mock::StateDot>;

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;
  const double dt{ 0.1 };
  const mock::State x0(0.1, 0);
  const mock::StateDot xdot(0.0, 0.0);
  const mock::State x1{ Integrator::integrate(x0, xdot, dt) };
  const mock::State x1_noise(x1 + mock::State(0.1, -0.1));

  const gtsam::Key key_x0{ gtsam::Symbol('x', 0) };
  const gtsam::Key key_x1{ gtsam::Symbol('x', 1) };
  const gtsam::Key key_xdot{ gtsam::Symbol('D', 1) };
  graph.emplace_shared<Integrator>(key_x1, key_x0, key_xdot, nullptr, dt);

  initial_values.insert(key_x0, x0);
  initial_values.insert(key_x1, x1_noise);
  initial_values.insert(key_xdot, xdot);
  graph.addPrior(key_x0, x0);
  graph.addPrior(key_xdot, xdot);
  gtsam::GaussNewtonParams params{};
  params.setRelativeErrorTol(1e-10);  // Need to set this lower than default
  params.setAbsoluteErrorTol(1e-10);
  gtsam::Values values = gtsam::GaussNewtonOptimizer(graph, initial_values, params).optimize();
  // values.print();

  const mock::State expected{ x1 };  // 0 + w*dt

  const mock::State result{ values.at<mock::State>(key_x1) };
  BOOST_REQUIRE_MESSAGE(expected.isApprox(result, 1e-3), EXPECTED_GOT(expected, result));
}

BOOST_AUTO_TEST_CASE(optimizer_find_xdot_test)
{
  using Integrator = prx::fg::euler_integration_factor_t<mock::State, mock::StateDot>;

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;
  const double dt{ 0.1 };
  const mock::State x0(0.1, 0);
  const mock::StateDot xdot(0.0, 0.0);
  const mock::StateDot xdot_noise(xdot + mock::State(0.1, -0.1));
  const mock::State x1{ Integrator::integrate(x0, xdot, dt) };

  const gtsam::Key key_x0{ gtsam::Symbol('x', 0) };
  const gtsam::Key key_x1{ gtsam::Symbol('x', 1) };
  const gtsam::Key key_xdot{ gtsam::Symbol('D', 1) };
  graph.emplace_shared<Integrator>(key_x1, key_x0, key_xdot, nullptr, dt);

  initial_values.insert(key_x0, x0);
  initial_values.insert(key_x1, x1);
  initial_values.insert(key_xdot, xdot_noise);
  graph.addPrior(key_x0, x0);
  graph.addPrior(key_x1, x1);
  gtsam::GaussNewtonParams params{};
  params.setRelativeErrorTol(1e-10);  // Need to set this lower than default
  params.setAbsoluteErrorTol(1e-10);
  gtsam::Values values = gtsam::GaussNewtonOptimizer(graph, initial_values, params).optimize();
  // values.print();

  const mock::StateDot expected{ xdot };  // 0 + w*dt

  const mock::StateDot result{ values.at<mock::StateDot>(key_xdot) };
  const double epsilon{ 0.01 };

  BOOST_REQUIRE_MESSAGE(std::abs(expected[0] - result[0]) < epsilon, EXPECTED_GOT(expected[0], result[0]));
  BOOST_REQUIRE_MESSAGE(std::abs(expected[1] - result[1]) < epsilon, EXPECTED_GOT(expected[1], result[1]));
}

BOOST_AUTO_TEST_CASE(optimizer_find_x0_with_dt_as_key_test)
{
  using Integrator = prx::fg::euler_integration_factor_t<mock::State, mock::StateDot, double>;

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;

  const double dt{ 0.1 };
  const mock::State x0(0.1, 0);
  const mock::State x0_noise(0.0, 0);
  const mock::StateDot xdot(0.0, 1.0);
  const mock::State x1{ Integrator::integrate(x0, xdot, dt) };

  const gtsam::Key key_x0{ gtsam::Symbol('x', 0) };
  const gtsam::Key key_x1{ gtsam::Symbol('x', 1) };
  const gtsam::Key key_dt{ gtsam::Symbol('t', 0) };
  const gtsam::Key key_xdot{ gtsam::Symbol('D', 0) };
  graph.emplace_shared<Integrator>(key_x1, key_x0, key_xdot, key_dt, nullptr);

  // NoiseModel dynamic_noise{};

  initial_values.insert(key_x0, x0_noise);
  initial_values.insert(key_x1, x1);
  initial_values.insert(key_dt, dt);
  initial_values.insert(key_xdot, xdot);

  graph.addPrior(key_xdot, xdot, gtsam::noiseModel::Isotropic::Sigma(2, 1e-5));
  graph.addPrior(key_x1, x1, gtsam::noiseModel::Isotropic::Sigma(2, 1e-5));
  graph.addPrior(key_dt, dt, gtsam::noiseModel::Isotropic::Sigma(1, 1e-5));

  gtsam::GaussNewtonParams params{};
  params.setRelativeErrorTol(1e-10);  // Need to set this lower than default
  params.setAbsoluteErrorTol(1e-10);
  gtsam::Values values = gtsam::GaussNewtonOptimizer(graph, initial_values, params).optimize();
  // values.print();

  const mock::State expected{ x0 };  // 0 + w*dt

  const mock::State result{ values.at<mock::State>(key_x0) };
  PRX_DBG_VARS(result);
  BOOST_REQUIRE_MESSAGE(expected.isApprox(result, 1e-3), EXPECTED_GOT(expected, result));
}

BOOST_AUTO_TEST_CASE(optimizer_find_dt_test)
{
  using Integrator = prx::fg::euler_integration_factor_t<mock::State, mock::StateDot, double>;

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;

  const double dt{ 0.1 };
  const double dt_noise{ 0.5 };
  const mock::State x0(0.1, 0);
  const mock::StateDot xdot(0.0, 1.0);
  const mock::State x1{ Integrator::integrate(x0, xdot, dt) };

  const gtsam::Key key_x0{ gtsam::Symbol('x', 0) };
  const gtsam::Key key_x1{ gtsam::Symbol('x', 1) };
  const gtsam::Key key_dt{ gtsam::Symbol('t', 0) };
  const gtsam::Key key_xdot{ gtsam::Symbol('D', 0) };
  graph.emplace_shared<Integrator>(key_x1, key_x0, key_xdot, key_dt, nullptr);

  // NoiseModel dynamic_noise{};

  initial_values.insert(key_x0, x0);
  initial_values.insert(key_x1, x1);
  initial_values.insert(key_dt, dt_noise);
  initial_values.insert(key_xdot, xdot);

  graph.addPrior(key_x0, x0);
  graph.addPrior(key_x1, x1);
  graph.addPrior(key_xdot, xdot);

  gtsam::GaussNewtonParams params{};
  params.setRelativeErrorTol(1e-10);  // Need to set this lower than default
  params.setAbsoluteErrorTol(1e-10);
  boost::shared_ptr<gtsam::GaussianFactorGraph> linear_fg{ graph.linearize(initial_values) };

  gtsam::Values values = gtsam::GaussNewtonOptimizer(graph, initial_values, params).optimize();
  // values.print();

  const double expected{ dt };

  const double result{ values.at<double>(key_dt) };
  // PRX_DBG_VARS(result);
  BOOST_REQUIRE_CLOSE(result, expected, 1e-3);
}