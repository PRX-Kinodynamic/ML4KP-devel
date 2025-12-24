#define BOOST_AUTO_TEST_MAIN first_order_derivative_test
#include <string>
#include <chrono>

#include <boost/test/unit_test.hpp>

#include "prx/utilities/math/continuous_algebraic_riccati_equation.hpp"
#include "prx/utilities/math/first_order_derivative.hpp"

using prx::math::I_min;
using prx::math::S;

struct _1D_model_test
{
  using VectorIn = double;  // Naming vector for backwards comp
  using VectorOut = Eigen::Vector<double, 2>;
  using Jacobian = Eigen::Matrix<double, 2, 1>;
  VectorOut operator()(const double& x) const
  {
    const double y0{ 5.0 * x + std::sin(x) };
    const double y1{ 5.0 * x * x };
    return VectorOut(y0, y1);
  }

  static Jacobian analytical(const double& x)
  {
    Jacobian out{ Jacobian::Zero() };

    out(0, 0) = 5.0 + std::cos(x);
    out(1, 0) = 2.0 * 5.0 * x;
    return out;
  }

  inline static VectorIn Zero(const Eigen::Index& dim)
  {
    return 0.0;
  }
  inline static VectorIn Random(const Eigen::Index& dim)
  {
    return Jacobian::Random()(0, 0);
  }
};

template <Eigen::Index Dim>
struct _2D_model_test
{
  using VectorIn = Eigen::Vector<double, Dim>;
  using VectorOut = Eigen::Vector<double, Dim>;
  // Could be a Vector, but lets make it explicit that in general, J is a matrix
  using Jacobian = Eigen::Matrix<double, Dim, Dim>;
  VectorOut operator()(const VectorIn& vector) const
  {
    const double x = vector[0];
    const double y = vector[1];
    VectorOut out{ Eigen::Vector2d::Zero() };
    out[0] = x * x * y;
    out[1] = 5.0 * x + std::sin(y);
    return out;
  }

  static Jacobian analytical(const VectorIn& vector)
  {
    const double x = vector[0];
    const double y = vector[1];
    Jacobian out{ Eigen::Matrix2d::Zero() };

    out(0, 0) = 2.0 * x * y;
    out(0, 1) = x * x;
    out(1, 0) = 5.0;
    out(1, 1) = std::cos(y);
    return out;
  }
  inline static VectorIn Zero(const Eigen::Index& dim)
  {
    return VectorIn::Zero(dim);
  }
  inline static VectorIn Random(const Eigen::Index& dim)
  {
    return VectorIn::Random(dim);
  }
};

template <Eigen::Index Dim>
struct _2D_model_multiple_inputs_test
{
  using VectorIn = Eigen::Vector<double, Dim>;
  using VectorIn1 = Eigen::Vector<double, Dim + 1>;
  using VectorIn2 = Eigen::Vector<double, Dim + 2>;
  using VectorOut = Eigen::Vector<double, Dim>;
  // Could be a Vector, but lets make it explicit that in general, J is a matrix
  using Jacobian = Eigen::Matrix<double, Dim, Dim>;

  VectorOut operator()(const VectorIn& vector, const VectorIn1& vector1, const VectorIn2& vector2) const
  {
    const double x = vector[0];
    const double y = vector[1];
    VectorOut out{ VectorOut::Zero() };
    out[0] = x * x * y;
    out[1] = 5.0 * x + std::sin(y);
    return out;
  }

  static Jacobian analytical(const VectorIn& vector)
  {
    const double x = vector[0];
    const double y = vector[1];
    Jacobian out{ Eigen::Matrix2d::Zero() };

    out(0, 0) = 2.0 * x * y;
    out(0, 1) = x * x;
    out(1, 0) = 5.0;
    out(1, 1) = std::cos(y);
    return out;
  }
  inline static VectorIn Zero(const Eigen::Index& dim)
  {
    return VectorIn::Zero(dim);
  }
  inline static VectorIn Random(const Eigen::Index& dim)
  {
    return VectorIn::Random(dim);
  }
};

template <Eigen::Index Dim>
struct _4D_2D_model_test
{
  using VectorIn = Eigen::Vector<double, Dim + 1>;
  using VectorOut = Eigen::Vector<double, Dim>;
  // Could be a Vector, but lets make it explicit that in general, J is a matrix
  using Jacobian = Eigen::Matrix<double, Dim, Dim + 1>;
  VectorOut operator()(const VectorIn& vector) const
  {
    const double x{ vector[0] };
    const double y{ vector[1] };
    const double z{ vector[2] };
    VectorOut out{ Eigen::Vector2d::Zero() };
    out[0] = x * y * z;
    out[1] = 5.0 * x + std::sin(y);
    return out;
  }

  static Jacobian analytical(const VectorIn& vector)
  {
    const double x{ vector[0] };
    const double y{ vector[1] };
    const double z{ vector[2] };
    Jacobian out{ Jacobian::Zero() };

    out(0, 0) = y * z;
    out(0, 1) = x * z;
    out(0, 2) = x * y;
    out(1, 0) = 5.0;
    out(1, 1) = std::cos(y);
    out(1, 2) = 0;
    return out;
  }
  inline static VectorIn Zero(const Eigen::Index& dim)
  {
    return VectorIn::Zero(dim);
  }
  inline static VectorIn Random(const Eigen::Index& dim)
  {
    return VectorIn::Random(dim);
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

  inline static VectorIn Zero(const Eigen::Index& dim)
  {
    return VectorIn::Zero(dim);
  }
  inline static VectorIn Random(const Eigen::Index& dim)
  {
    return VectorIn::Random(dim);
  }
};

template <S s, I_min i_min, typename Model>
void model_test(const double h, const double tolerance_constant, const Eigen::Index dim_in, const Eigen::Index dim_out)
{
  using VectorIn = typename Model::VectorIn;
  using Jacobian = typename Model::Jacobian;
  using Derivative = prx::math::first_order_derivative_t<Model, VectorIn, s, i_min>;
  // Error is: O(h^{s-1}). Obviously, the actual error might be a somewhat higher...
  // so we test using tolerance = C*h^{s-1}
  const double tolerance{ tolerance_constant * std::pow(h, s - 1) };
  const int total_evaluations{ 1 };

  VectorIn x_in{ Model::Zero(dim_in) };
  Derivative derivative(h, dim_in, dim_out);
  for (int i = 0; i < total_evaluations; ++i)
  {
    x_in = 100 * Model::Random(dim_in);
    Jacobian numerical_derivative = derivative(x_in);
    Jacobian analytical_derivative = Model::analytical(x_in);

    std::stringstream ss;
    ss << "Numerical: \n:" << numerical_derivative << "\nAnalytical:\n" << analytical_derivative << std::endl;
    BOOST_CHECK_MESSAGE(numerical_derivative.isApprox(analytical_derivative, tolerance), ss.str());
  }
}

template <typename Model>
void run_full_derivative_table(const Eigen::Index dim_in, const Eigen::Index dim_out)
{
  // Error is O(h^(s-1)), in practice this deriv is bad so giving a cte of 100.
  model_test<2, 0, Model>(0.001, 100, dim_in, dim_out);
  model_test<2, -1, Model>(0.001, 100, dim_in, dim_out);

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
}

BOOST_AUTO_TEST_CASE(model_1D_test)
{
  using Model = _1D_model_test;
  const int DimIn{ 1 };
  const int DimOut{ 2 };
  auto start = std::chrono::steady_clock::now();

  run_full_derivative_table<Model>(DimIn, DimOut);
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
}

BOOST_AUTO_TEST_CASE(model_2D_test)
{
  using Model = _2D_model_test<2>;
  const int DimIn{ 2 };
  const int DimOut{ 2 };
  auto start = std::chrono::steady_clock::now();

  run_full_derivative_table<Model>(DimIn, DimOut);
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
}

BOOST_AUTO_TEST_CASE(model_4D_2D_test)
{
  using Model = _4D_2D_model_test<2>;
  const int DimIn{ 3 };
  const int DimOut{ 2 };
  auto start = std::chrono::steady_clock::now();

  run_full_derivative_table<Model>(DimIn, DimOut);
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
}

BOOST_AUTO_TEST_CASE(model_2D_dynamic_test)
{
  using Model = _2D_model_test<Eigen::Dynamic>;
  const int DimIn{ 2 };
  const int DimOut{ 2 };
  auto start = std::chrono::steady_clock::now();
  run_full_derivative_table<Model>(DimIn, DimOut);

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_seconds = end - start;
  std::cout << "elapsed time: " << elapsed_seconds.count() << "s\n";
}

BOOST_AUTO_TEST_CASE(distance_derivative_test)
{
  const int DimIn{ 3 };
  const int DimOut{ 1 };
  using Model = distance_model_test<DimIn>;

  run_full_derivative_table<Model>(DimIn, DimOut);
}

BOOST_AUTO_TEST_CASE(model_multiple_inputs_test)
{
  const int DimIn{ 2 };
  const int DimOut{ 2 };
  using Model = _2D_model_multiple_inputs_test<DimIn>;
  // run_full_derivative_table<Model>(DimIn, DimOut);

  const prx::math::S s{ 3 };
  const prx::math::I_min i_min{ -1 };
  const double h{ 0.01 };

  using Jacobian = typename Model::Jacobian;
  using Derivative =
      prx::math::first_order_derivative_t<Model, Model::VectorIn, s, i_min, Model::VectorIn1, Model::VectorIn2>;
  // Error is: O(h^{s-1}). Obviously, the actual error might be a somewhat higher...
  // so we test using tolerance = C*h^{s-1}
  const double tolerance{ std::pow(h, s - 1) };
  const int total_evaluations{ 1 };

  Model::VectorIn x_in{ Model::Zero(DimIn) };
  Model::VectorIn1 x1_in{ Model::VectorIn1::Zero() };
  Model::VectorIn2 x2_in{ Model::VectorIn2::Zero() };

  Derivative derivative(h, DimIn, DimOut);

  for (int i = 0; i < total_evaluations; ++i)
  {
    x_in = 100 * Model::Random(DimIn);
    x1_in = 100 * Model::VectorIn1::Random();
    x2_in = 100 * Model::VectorIn2::Random();
    Jacobian numerical_derivative = derivative(x_in, x1_in, x2_in);
    Jacobian analytical_derivative = Model::analytical(x_in);

    std::stringstream ss;
    ss << "Numerical: \n:" << numerical_derivative << "\nAnalytical:\n" << analytical_derivative << std::endl;
    BOOST_CHECK_MESSAGE(numerical_derivative.isApprox(analytical_derivative, tolerance), ss.str());
  }
}