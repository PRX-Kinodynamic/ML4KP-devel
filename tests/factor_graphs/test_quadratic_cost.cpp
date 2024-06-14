#define BOOST_AUTO_TEST_MAIN quadratic_cost_factor_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/defs.hpp"
#include "prx/factor_graphs/factors/quadratic_cost_factor.hpp"

#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/geometry/SOn.h>

BOOST_AUTO_TEST_CASE(propagate_without_derivatives_test)
{
  using State = Eigen::Vector<double, 2>;
  using Cost = Eigen::Matrix<double, 2, 2>;

  const gtsam::Key key_x{ gtsam::Symbol('x', 0) };

  prx::fg::quadratic_cost_factor_t<State> qcf(key_x, Cost::Identity(), nullptr);

  const State x0{ State::Zero() };
  const State x1{ State(2, 0) };
  const Eigen::VectorXd x0_error{ qcf.evaluateError(x0) };
  const Eigen::VectorXd x1_error{ qcf.evaluateError(x1) };

  const double x0_expected{ 0 };
  const double x1_expected{ 4 };

  BOOST_REQUIRE_CLOSE(x0_error[0], x0_expected, 1e-3);
  BOOST_REQUIRE_CLOSE(x1_error[0], x1_expected, 1e-3);
}