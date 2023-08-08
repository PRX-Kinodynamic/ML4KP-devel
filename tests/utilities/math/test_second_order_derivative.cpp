#define BOOST_AUTO_TEST_MAIN first_order_derivative_test

#include <string>
#include <chrono>

#include <boost/test/unit_test.hpp>

#include "prx/utilities/math/continuous_algebraic_riccati_equation.hpp"
#include "prx/utilities/math/second_order_derivative.hpp"

using prx::math::I_min;
using prx::math::S;

// Templated to be able to test Dim=Dynamic
template <Eigen::Index Dim>
struct _1D_model_test
{
  // f(x)=x^3
  // f'(x)=3x^2
  // f''(x)=6x
  using VectorIn = Eigen::Vector<double, Dim>;
  using VectorOut = Eigen::Vector<double, Dim>;
  using Derivative2nd = Eigen::Matrix<double, Dim, Dim>;
  VectorOut operator()(const VectorIn& x) const
  {
    VectorOut y{ VectorOut::Zero(1) };
    y << std::pow(x[0], 3);
    return y;
  }
  static Derivative2nd analytical(const VectorIn& x)
  {
    Derivative2nd out{ Eigen::Matrix<double, 1, 1>::Zero() };
    out << 6.0 * x[0];
    return out;
  }
};

template <Eigen::Index DimIn, Eigen::Index DimOut>
struct _2D_model_test
{
  using VectorIn = Eigen::Vector<double, DimIn>;
  using VectorOut = Eigen::Vector<double, DimOut>;
  using Derivative2nd = Eigen::Matrix<double, DimIn, DimIn>;
  VectorOut operator()(const VectorIn& vector) const
  {
    const double x = vector[0];
    const double y = vector[1];
    VectorOut out{ Eigen::Vector<double, 1>::Zero() };
    out[0] = 5.0 * std::pow(x, 3) + std::sin(y);
    return out;
  }

  static Derivative2nd analytical(const VectorIn& vector)
  {
    const double x = vector[0];
    const double y = vector[1];
    Derivative2nd out{ Eigen::Matrix2d::Zero() };
    // (10 tan(x) sec^2(x) | 0
    // 0 | -sin(y))
    out(0, 0) = 30.0 * x;
    out(0, 1) = 0.0;
    out(1, 0) = 0.0;
    out(1, 1) = -std::sin(y);
    return out;
  }
};

template <Eigen::Index Dim>
struct distance_model_test
{
  using VectorIn = Eigen::Vector<double, Dim>;
  using VectorOut = Eigen::Vector<double, 1>;
  // Could be a Vector, but lets make it explicit that in general, J is a matrix
  using Jacobian = Eigen::Matrix<double, Dim, 1>;

  VectorOut operator()(const VectorIn& vector) const
  {
    return VectorOut{ vector.norm() };
  }

  static Jacobian analytical(const VectorIn& vector)
  {
    const double norm{ vector.norm() };
    const Jacobian jac{ vector / norm };
    return jac;
  }
};

template <S s, I_min i_min, typename Model>
void model_test(const double h, const double tolerance_constant, const Eigen::Index dim_in, const Eigen::Index dim_out)
{
  using VectorIn = typename Model::VectorIn;
  using Derivative2nd = typename Model::Derivative2nd;
  using Derivative = prx::math::second_order_derivative_t<Model, VectorIn, s, i_min>;
  // Error is: O(h^{s-1}). Obviusly, the actual error might be a somewhat higher...
  // so we test using tolerance = C*h^{s-2}
  const double tolerance{ tolerance_constant * std::pow(h, s - 2) };
  const int total_evaluations{ 1 };

  VectorIn x_in{ VectorIn::Zero(dim_in) };
  Derivative derivative(h, dim_in, dim_out);
  for (int i = 0; i < total_evaluations; ++i)
  {
    x_in = 100 * VectorIn::Random(dim_in);
    Derivative2nd numerical_derivative = derivative(x_in);
    Derivative2nd analytical_derivative = Model::analytical(x_in);

    std::stringstream ss;
    ss << "s: " << static_cast<int>(s) << "\ti_min " << static_cast<int>(i_min) << "\nx_in: " << x_in
       << "\nNumerical: \n:" << numerical_derivative << "\nAnalytical:\n"
       << analytical_derivative << std::endl;
    BOOST_CHECK_MESSAGE(numerical_derivative.isApprox(analytical_derivative, tolerance), ss.str());
  }
}

template <typename Model>
void run_full_derivative_table(const Eigen::Index dim_in, const Eigen::Index dim_out)
{
  // Error is O(h^(s-1))
  model_test<3, 0, Model>(0.01, 1, dim_in, dim_out);
  model_test<3, -1, Model>(0.01, 1, dim_in, dim_out);
  model_test<3, -2, Model>(0.01, 1, dim_in, dim_out);
  model_test<4, 0, Model>(0.1, 1, dim_in, dim_out);
  model_test<4, -1, Model>(0.1, 1, dim_in, dim_out);
  model_test<4, -2, Model>(0.1, 1, dim_in, dim_out);
  model_test<4, -3, Model>(0.1, 1, dim_in, dim_out);
  model_test<5, 0, Model>(0.5, 1, dim_in, dim_out);
  model_test<5, -1, Model>(0.5, 1, dim_in, dim_out);
  model_test<5, -2, Model>(0.5, 1, dim_in, dim_out);
  model_test<5, -3, Model>(0.5, 1, dim_in, dim_out);
  model_test<5, -4, Model>(0.5, 1, dim_in, dim_out);
  model_test<6, 0, Model>(0.75, 1, dim_in, dim_out);
  model_test<6, -1, Model>(0.75, 1, dim_in, dim_out);
  model_test<6, -4, Model>(0.75, 1, dim_in, dim_out);
  model_test<6, -5, Model>(0.75, 1, dim_in, dim_out);
}
BOOST_AUTO_TEST_CASE(model_1D_test)
{
  using Model = _1D_model_test<1>;
  const int DimIn{ 1 };
  const int DimOut{ 1 };
  auto start = std::chrono::steady_clock::now();

  run_full_derivative_table<Model>(DimIn, DimOut);
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
}

BOOST_AUTO_TEST_CASE(model_1D_dynamic_test)
{
  using Model = _1D_model_test<Eigen::Dynamic>;
  const int DimIn{ 1 };
  const int DimOut{ 1 };
  auto start = std::chrono::steady_clock::now();

  run_full_derivative_table<Model>(DimIn, DimOut);
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
}