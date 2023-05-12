#pragma once
#include <array>
#include <numeric>
#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/utilities/math/math_functions.hpp"
// #include "prx/factor_graphs/utilities/prx_symbols.hpp"
#include "prx/factor_graphs/utilities/symbols_factory.hpp"
#include "prx/simulation/plants/types/linear_time_variant.hpp"
#include "prx/utilities/math/first_order_derivative.hpp"

namespace prx
{
namespace fg
{
namespace default_functions
{

template <typename X0, typename X1>
std::function<X0(const X1&)> f2 = [](const X1&) {
  PRX_NOT_IMPLEMENTED;
  return X0::Zero();
};
template <typename X0, typename X1, typename X2>
std::function<X0(const X1&, const X2&)> f3 = [](const X1&, const X2&) {
  PRX_NOT_IMPLEMENTED;
  return X0::Zero();
};
}  // namespace default_functions
/**
 * @brief      Factor that implementes x0 ~= f(x1, x2), where xi is an Eigen::Vector<double, DIM_i>
 *
 * @tparam     DIM_0  Dimension of x0
 * @tparam     DIM_1  Dimension of x1
 */
template <Eigen::Index DIM_0, Eigen::Index DIM_1>
class function_2factor_t : public gtsam::NoiseModelFactor2<Eigen::Vector<double, DIM_0>, Eigen::Vector<double, DIM_1>>
{
  using X0 = Eigen::Vector<double, DIM_0>;
  using X1 = Eigen::Vector<double, DIM_1>;
  using Base = gtsam::NoiseModelFactor2<X0, X1>;

  template <typename Xi>
  using Derivative = std::function<X0(const Xi&)>;
  using Function = std::function<X0(const X1&)>;

public:
  function_2factor_t(const gtsam::noiseModel::Base::shared_ptr& cost_model, gtsam::Key x0_key, gtsam::Key x1_key,
                     const Function& function = default_functions::f2<X0, X1>)
    : Base(cost_model, x0_key, x1_key), _function(function), derivative_x0(0.01), derivative_x1(0.01)
  {
  }

  virtual Eigen::VectorXd evaluateError(const X0& x0, const X1& x1, boost::optional<Eigen::MatrixXd&> H1 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H2 = boost::none) const override
  {
    const Eigen::VectorXd error{ compute_error(x0, x1) };
    if (H1)
    {
      derivative_x0._model = [&](const X0& _x0) { return compute_error(_x0, x1); };
      *H1 = derivative_x0(x0);
    }

    if (H2)
    {
      derivative_x1._model = [&](const X1& _x1) { return compute_error(x0, _x1); };
      *H2 = derivative_x1(x1);
    }

    return error;
  }

  Eigen::VectorXd compute_error(const X0& x0, const X1& x1) const
  {
    return x0 - _function(x1);
  }

  Function _function;

private:
  mutable math::first_order_derivative_t<Derivative<X0>, X0, 4> derivative_x0;
  mutable math::first_order_derivative_t<Derivative<X1>, X1, 4> derivative_x1;
};

/**
 * @brief      Factor that implementes x0 ~= f(x1, x2), where xi is an Eigen::Vector<double, DIM_i>
 *
 * @tparam     DIM_0  Dimension of x0
 * @tparam     DIM_1  Dimension of x1
 * @tparam     DIM_2  Dimension of x2
 */
template <Eigen::Index DIM_0, Eigen::Index DIM_1, Eigen::Index DIM_2>
class function_3factor_t : public gtsam::NoiseModelFactor3<Eigen::Vector<double, DIM_0>, Eigen::Vector<double, DIM_1>,
                                                           Eigen::Vector<double, DIM_2>>
{
  using X0 = Eigen::Vector<double, DIM_0>;
  using X1 = Eigen::Vector<double, DIM_1>;
  using X2 = Eigen::Vector<double, DIM_2>;
  using Base = gtsam::NoiseModelFactor3<X0, X1, X2>;

  template <typename Xi>
  using Derivative = std::function<X0(const Xi&)>;
  using Function = std::function<X0(const X1&, const X2&)>;

public:
  function_3factor_t(const gtsam::noiseModel::Base::shared_ptr& cost_model, gtsam::Key x0_key, gtsam::Key x1_key,
                     gtsam::Key x2_key, const Function& function = default_functions::f3<X0, X1, X2>)
    : Base(cost_model, x0_key, x1_key, x2_key)
    , _function(function)
    , derivative_x0(0.01)
    , derivative_x1(0.01)
    , derivative_x2(0.01)
  {
  }

  virtual Eigen::VectorXd evaluateError(const X0& x0, const X1& x1, const X2& x2,
                                        boost::optional<Eigen::MatrixXd&> H1 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H2 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H3 = boost::none) const override
  {
    const Eigen::VectorXd error{ compute_error(x0, x1, x2) };
    if (H1)
    {
      derivative_x0._model = [&](const X0& _x0) { return compute_error(_x0, x1, x2); };
      *H1 = derivative_x0(x0);
    }

    if (H2)
    {
      derivative_x1._model = [&](const X1& _x1) { return compute_error(x0, _x1, x2); };
      *H2 = derivative_x1(x1);
    }

    if (H3)
    {
      derivative_x2._model = [&](const X2& _x2) { return compute_error(x0, x1, _x2); };
      *H3 = derivative_x2(x2);
    }

    return error;
  }

  Eigen::VectorXd compute_error(const X0& x0, const X1& x1, const X2& x2) const
  {
    return x0 - _function(x1, x2);
  }

  Function _function;

private:
  mutable math::first_order_derivative_t<Derivative<X0>, X0, 4> derivative_x0;
  mutable math::first_order_derivative_t<Derivative<X1>, X1, 4> derivative_x1;
  mutable math::first_order_derivative_t<Derivative<X2>, X2, 4> derivative_x2;
};
}  // namespace fg
}  // namespace prx