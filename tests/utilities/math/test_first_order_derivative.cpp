#define BOOST_AUTO_TEST_MAIN first_order_derivative_test
#include <string>
#include <boost/test/unit_test.hpp>
#include "prx/utilities/math/continuous_algebraic_riccati_equation.hpp"
#include "prx/utilities/math/first_order_derivative.hpp"

using prx::math::I_min;
using prx::math::S;
using Model2D = std::function<Eigen::Vector2d(Eigen::Vector2d)>;
using State2D = Eigen::Vector2d;
using Matrix2D = Eigen::Matrix2d;
Model2D model_2d = [](State2D in) {
  const double x = in[0];
  const double y = in[1];

  State2D out;
  out[0] = x * x * y;
  out[1] = 5.0 * x + std::sin(y);
  return out;
};

std::function<Matrix2D(State2D)> analytical_jacobian = [](State2D in) {
  const double x = in[0];
  const double y = in[1];
  Matrix2D out;

  out(0, 0) = 2.0 * x * y;
  out(0, 1) = x * x;
  out(1, 0) = 5.0;
  out(1, 1) = std::cos(y);
  return out;
};

template <S s, I_min i_min>
void model_2d_test(const double h, const double tolerance_constant)
{
  // Error is: O(h^{s-1}). Obviusly, the actual error might be a somewhat higher...
  // so we test using tolerance = C*h^{s-1}
  const double tolerance{ tolerance_constant * std::pow(h, s - 1) };
  const int total_evaluations{ 1000 };

  State2D x_in_1{ 0.0, 0.0 };
  prx::math::first_order_derivative_t<Model2D, State2D, s, i_min> derivative(model_2d, h);
  for (int i = 0; i < total_evaluations; ++i)
  {
    x_in_1 = 100 * State2D::Random();
    Eigen::Matrix2d numerical_derivative = derivative(x_in_1);
    Eigen::Matrix2d analytical_derivative = analytical_jacobian(x_in_1);

    std::stringstream ss;
    ss << "Numerical: \n:" << numerical_derivative << "\nAnalytical:\n" << analytical_derivative << std::endl;
    BOOST_CHECK_MESSAGE(numerical_derivative.isApprox(analytical_derivative, tolerance), ss.str());
  }
}

BOOST_AUTO_TEST_CASE(model_2D_test)
{
  model_2d_test<2, 0>(0.001, 100);   // Error is O(h^(s-1)), in practice this deriv is bad so giving a cte of 100.
  model_2d_test<2, -1>(0.001, 100);  // Error is O(h^(s-1)), in practice this deriv is bad so giving a cte of 100.
  model_2d_test<3, 0>(0.01, 1);
  model_2d_test<3, -1>(0.01, 1);
  model_2d_test<3, -2>(0.01, 1);
  model_2d_test<4, 0>(0.1, 1);
  model_2d_test<4, -1>(0.1, 1);
  model_2d_test<4, -2>(0.1, 1);
  model_2d_test<4, -3>(0.1, 1);
  model_2d_test<5, 0>(0.5, 1);
  model_2d_test<5, -1>(0.5, 1);
  model_2d_test<5, -2>(0.5, 1);
  model_2d_test<5, -3>(0.5, 1);
  model_2d_test<5, -4>(0.5, 1);
}
