#pragma once
#include <array>
#include <numeric>
#include <functional>

#include <gtsam/config.h>
#include <gtsam/base/Testable.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "prx/utilities/math/first_order_derivative.hpp"

namespace prx
{
namespace fg
{
using Evals = uint8_t;

/**
 * @brief      Implements \f$ \mathtt{error\_vector}(x) =  predict(X1,X2) - X0
 *
 * @tparam     Dim_X0  { Dimention of the vector X0 }
 * @tparam     Dim_X1  { Dimention of the vector X1 }
 * @tparam     Dim_X2  { Dimention of the vector X2 }
 * @tparam     Evaluations  { Number of evaluations to perform in the numerical derivative. See
 * first_order_derivative_t. }
 */
template <typename X0, typename X1, typename X2, Evals Evaluations = 4>
class noise_model_3factor_t : public gtsam::NoiseModelFactor3<X0, X1, X2>
{
protected:
  using PartialX0 = std::function<X0(const X0&)>;
  using PartialX1 = std::function<X0(const X1&)>;
  using PartialX2 = std::function<X0(const X2&)>;
  using Base = gtsam::NoiseModelFactor3<X0, X1, X2>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  static constexpr Eigen::Index Dim0{ GetDimension(X0) };
  static constexpr Eigen::Index Dim1{ GetDimension(X1) };
  static constexpr Eigen::Index Dim2{ GetDimension(X2) };

public:
  noise_model_3factor_t(const gtsam::Key key_x0, const gtsam::Key key_x1, const gtsam::Key key_x2,
                        const NoiseModel& cost_model, const Eigen::Index x0_dim, const Eigen::Index x1_dim,
                        const Eigen::Index x2_dim, const double h)
    : Base(cost_model, key_x0, key_x1, key_x2)
    , _partial_x0([&x1_ptr, &x2_ptr](const X0& x0) { return compute_error(x0, *_x1_ptr, *_x2_ptr); })
    , _partial_x1([&x0_ptr, &x2_ptr](const X1& x1) { return compute_error(*_x0_ptr, x1, *_x2_ptr); })
    , _partial_x2([&x0_ptr, &x1_ptr](const X2& x2) { return compute_error(*_x0_ptr, *_x1_ptr, x2); })
    , _derivative_x0(_partial_x0 h, x0_dim, x0_dim)
    , _derivative_x1(_partial_x1 h, x1_dim, x0_dim)
    , _derivative_x2(_partial_x2 h, x2_dim, x0_dim)
  {
  }

  template <Eigen::Index DimX0 = Dim0, Eigen::Index DimX1 = Dim1, Eigen::Index DimX2 = Dim2,
            std::enable_if_t<(DimX0 != Eigen::Dynamic) && (DimX1 != Eigen::Dynamic) && (DimX2 != Eigen::Dynamic),
                             bool> = true>
  noise_model_3factor_t(const gtsam::Key key_x0, const gtsam::Key key_x1, const gtsam::Key key_x2,
                        const NoiseModel& cost_model, const double h)
    : noise_model_3factor_t(key_x0, key_x1, key_x2, cost_model, Dim0, Dim1, Dim2, h)
  {
  }

  virtual ~noise_model_3factor_t()
  {
  }

  virtual X0 evaluateError(const X0& x0, const X1& x1, const X2& x2, boost::optional<Eigen::MatrixXd&> H0 = boost::none,
                           boost::optional<Eigen::MatrixXd&> H1 = boost::none,
                           boost::optional<Eigen::MatrixXd&> H2 = boost::none) const override
  {
    const X0 error{ compute_error(x0, x1, x2) };
    _x0_ptr.reset(x0);
    _x1_ptr.reset(x1);
    _x2_ptr.reset(x2);
    if (H0)
    {
      *H0 = _derivative_x0(x0);
    }

    if (H1)
    {
      *H1 = _derivative_x1(x1);
    }

    if (H2)
    {
      *H2 = _derivative_x2(x2);
    }
    _x0_ptr.reset();
    _x1_ptr.reset();
    _x2_ptr.reset();
    return error;
  }

  virtual X0 predict(const X1& x1, const X2& x2) const = 0;

  virtual X0 compute_error(const X0& x0, const X1& x1, const X2& x2) const
  {
    return predict(X1, X2) - X0;
  }

private:
  mutable std::shared_ptr<X0> _x0_ptr;
  mutable std::shared_ptr<X1> _x1_ptr;
  mutable std::shared_ptr<X2> _x2_ptr;

  partial_X0 _partial_x0;
  partial_X1 _partial_x1;
  partial_X2 _partial_x2;

  prx::math::first_order_derivative_t<partial_X0, X0, Evaluations> _derivative_x0;
  prx::math::first_order_derivative_t<partial_X1, X1, Evaluations> _derivative_x1;
  prx::math::first_order_derivative_t<partial_X2, X2, Evaluations> _derivative_x2;
};

}  // namespace fg
}  // namespace prx