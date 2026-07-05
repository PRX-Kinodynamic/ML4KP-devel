#define BOOST_AUTO_TEST_MAIN se3_test
#include <string>
#include <boost/test/unit_test.hpp>
#include <gtsam/base/numericalDerivative.h>
#include "prx/utilities/defs.hpp"
#include "prx/factor_graphs/factors/fxu_factor.hpp"
#include "prx/simulation/dynamical_system.hpp"

namespace prx
{
class mock_plant_t;
template <>
struct dynamical_system_traits<mock_plant_t>
{
  // clang-format off
  enum {StateDimension = 2}; 
  enum {ControlDimension = 1}; 
  enum {ParametersDimension = 1}; 
  enum {ObservationDimension = 1};
  // clang-format on

  using State = Eigen::Vector2d;
  using Control = double;
  using Parameters = double;
  using Observation = double;

  using StateDot = Eigen::Vector<double, StateDimension>;
};
struct mock_plant_t : public prx::dynamical_system_t<mock_plant_t>
{
  void initialize() {};

  State propagate(const State& x0, const Control& u0, const double& dt, OptJacX Hx = nullptr, OptJacU Hu = nullptr,
                  OptJacDT Hdt = nullptr)
  {
    const State x1{ x0 * std::sin(u0) };
    if (Hx)
    {
      *Hx = Eigen::Matrix2d::Identity() * std::sin(u0);
    }
    if (Hu)
    {
      *Hu = x0 * std::cos(u0);
    }
    return x1;
  }
};

}  // namespace prx

BOOST_AUTO_TEST_CASE(test_derivatives)
{
  using State = prx::mock_plant_t::State;
  using Control = prx::mock_plant_t::Control;

  State x0_{ State(0, 1) };
  State x1_{ State(1, 0) };
  Control u0_{ 0.5 };

  // Check jacobians
  Eigen::MatrixXd actualHx0, expectedHx0;
  Eigen::MatrixXd actualHx1, expectedHx1;
  Eigen::MatrixXd actualHu0, expectedHu0;

  std::shared_ptr<prx::mock_plant_t> plant{ std::make_shared<prx::mock_plant_t>() };
  prx::fg::fxu_factor_t<State, Control, std::shared_ptr<prx::mock_plant_t>> factor(0, 1, 2, nullptr, 0.1, plant);

  std::function<gtsam::Vector(const State&, const Control&, const State&)> err_proxy =
      [&factor](const State& x0, const Control& u0, const State& x1) { return factor.evaluateError(x0, u0, x1); };

  factor.evaluateError(x0_, u0_, x1_, actualHx0, actualHu0, actualHx1);
  expectedHx0 = gtsam::numericalDerivative31(err_proxy, x0_, u0_, x1_);
  expectedHu0 = gtsam::numericalDerivative32(err_proxy, x0_, u0_, x1_);
  expectedHx1 = gtsam::numericalDerivative33(err_proxy, x0_, u0_, x1_);

  PRX_DBG_VARS(expectedHx0);
  PRX_DBG_VARS(actualHx0);

  PRX_DBG_VARS(expectedHu0);
  PRX_DBG_VARS(actualHu0);

  PRX_DBG_VARS(expectedHx1);
  PRX_DBG_VARS(actualHx1);

  const double tolerance{ 0.01 };
  const bool expectedHx0_isApprox_actualHx0{ expectedHx0.isApprox(actualHx0, tolerance) };
  const bool expectedHx1_isApprox_actualHx1{ expectedHx1.isApprox(actualHx1, tolerance) };
  const bool expectedHu0_isApprox_actualHu0{ expectedHu0.isApprox(actualHu0, tolerance) };

  BOOST_REQUIRE(expectedHx0_isApprox_actualHx0);
  BOOST_REQUIRE(expectedHx1_isApprox_actualHx1);
  BOOST_REQUIRE(expectedHu0_isApprox_actualHu0);
}
