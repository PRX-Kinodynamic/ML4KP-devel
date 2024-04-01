#define BOOST_AUTO_TEST_MAIN lie_integrator_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/defs.hpp"
#include "prx/factor_graphs/lie_groups/lie_integrator.hpp"

namespace mock
{
// using X = Eigen::Vector<double, 3>;
using Rdot = Eigen::Vector<double, 1>;
struct SO2
{
  static constexpr Eigen::Index RowsAtCompileTime = 2;
  SO2(double theta) : _R(Eigen::Matrix2d::Zero())
  {
    _R(0, 0) = std::cos(theta);
    _R(0, 1) = -std::sin(theta);
    _R(1, 0) = std::sin(theta);
    _R(1, 1) = std::cos(theta);
  }

  SO2(const Eigen::Matrix2d& other) : _R(other)
  {
  }
  SO2(const SO2& other) : _R(other._R)
  {
  }
  // Composition is +
  SO2 operator*(const double& other) const
  {
    const SO2 res{ other * _R };
    return res;
  }
  SO2 operator*(const SO2& other) const
  {
    const SO2 so2{ _R * other._R };
    return so2;
  }
  template <typename Rdot>
  static SO2 expmap(const Rdot& wz)
  {
    SO2 so2(wz[0]);
    // PRX_DEBUG_VAR_1(so2._R);
    // so2 = so2 * wz[0];
    // PRX_DEBUG_VAR_2(wz, so2._R);
    return so2;
  }
  Eigen::Matrix2d _R;
};

}  // namespace mock

BOOST_AUTO_TEST_CASE(propagete_without_derivatives_test)
{
  using Integrator = prx::fg::lie_integrator_t<mock::SO2, mock::Rdot>;
  const double dt{ 0.1 };
  const Integrator integrator{};
  const double angle0{ 0 };
  const double angle_dot0{ 1 };
  const mock::SO2 x0{ angle0 };
  // const mock::SO2 xdot0{ mock::SO2(angle_dot0) };
  const mock::Rdot xdot0(angle_dot0);

  const mock::SO2 x1{ integrator(x0, xdot0, dt) };

  const double angle_1{ angle0 + angle_dot0 * dt };
  const mock::SO2 x1_p{ angle_1 };

  const Eigen::Matrix2d expected{ x1_p._R };
  const Eigen::Matrix2d result{ x1._R };
  BOOST_REQUIRE_MESSAGE(expected.isApprox(result), EXPECTED_GOT(expected, result));
}

BOOST_AUTO_TEST_CASE(propagete_without_derivatives_multiple_steps_test)
{
  using Integrator = prx::fg::lie_integrator_t<mock::SO2, mock::Rdot>;
  const double dt{ 0.1 };
  const Integrator integrator{};
  const double angle0{ 0 };
  const mock::SO2 x0{ angle0 };
  const mock::Rdot xdot0(1);
  const mock::Rdot xdot1(2);
  const mock::Rdot xdot2(3);

  const mock::SO2 x1{ integrator(x0, xdot0, dt) };
  const mock::SO2 x2{ integrator(x1, xdot1, dt) };
  const mock::SO2 x3{ integrator(x2, xdot2, dt) };

  const double angle1{ angle0 + xdot0[0] * dt };
  const double angle2{ angle1 + xdot1[0] * dt };
  const double angle3{ angle2 + xdot2[0] * dt };
  const mock::SO2 x1_p{ angle1 };
  const mock::SO2 x2_p{ angle2 };
  const mock::SO2 x3_p{ angle3 };

  const Eigen::Matrix2d expected{ x3_p._R };
  const Eigen::Matrix2d result{ x3._R };
  BOOST_REQUIRE_MESSAGE(expected.isApprox(result), EXPECTED_GOT(expected, result));
}