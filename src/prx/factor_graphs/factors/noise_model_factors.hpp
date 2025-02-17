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

template <typename X0, Evals Evaluations = 4>
class noise_model_1factor_t : public gtsam::NoiseModelFactor3<X0>
{
public:
  static constexpr Eigen::Index Dim0{ gtsam::traits<X0>::dimension };

  using Error = Eigen::Vector<double, Dim0>;
  using PartialX0 = std::function<Error(const X0&)>;
  using FirstOrderDerivativeX0 = prx::math::first_order_derivative_t<PartialX0, X0, Evaluations>;
  using Base = gtsam::NoiseModelFactor3<X0>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

public:
  noise_model_1factor_t(const gtsam::Key key_x0, const NoiseModel& cost_model, const Eigen::Index x0_dim,
                        const double h)
    : Base(cost_model, key_x0)
    , _x0_ptr()
    , _partial_x0([&](const X0& x0) { return compute_error(x0); })
    , _derivative_x0(_partial_x0, h, x0_dim, x0_dim)
  {
  }

  template <Eigen::Index DimX0 = Dim0, std::enable_if_t<(DimX0 != Eigen::Dynamic), bool> = true>
  noise_model_1factor_t(const gtsam::Key key_x0, const NoiseModel& cost_model, const double h)
    : noise_model_1factor_t(key_x0, cost_model, Dim0, h)
  {
  }

  virtual ~noise_model_1factor_t()
  {
  }

  virtual Eigen::VectorXd evaluateError(const X0& x0, boost::optional<Eigen::MatrixXd&> H0 = boost::none) const override
  {
    const Error error{ compute_error(x0) };
    _x0_ptr.reset(&x0);
    if (H0)
    {
      *H0 = _derivative_x0(x0);
    }

    _x0_ptr.release();
    return error;
  }

  virtual X0 predict() const {};  // predict has default impl because may be trivial

  virtual Error compute_error(const X0& x0) const
  {
    return predict() - x0;
  }

private:
  mutable std::unique_ptr<const X0> _x0_ptr;

  const PartialX0 _partial_x0;

  const FirstOrderDerivativeX0 _derivative_x0;
};

template <typename X0, typename X1, Evals Evaluations = 4>
class noise_model_2factor_t : public gtsam::NoiseModelFactor3<X0, X1>
{
public:
  static constexpr Eigen::Index Dim0{ gtsam::traits<X0>::dimension };
  static constexpr Eigen::Index Dim1{ gtsam::traits<X1>::dimension };

  using Error = Eigen::Vector<double, Dim0>;
  using PartialX0 = std::function<Error(const X0&)>;
  using PartialX1 = std::function<Error(const X1&)>;
  using FirstOrderDerivativeX0 = prx::math::first_order_derivative_t<PartialX0, X0, Evaluations, -1>;
  using FirstOrderDerivativeX1 = prx::math::first_order_derivative_t<PartialX1, X1, Evaluations, -1>;
  using Base = gtsam::NoiseModelFactor3<X0, X1>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

public:
  noise_model_2factor_t(const gtsam::Key key_x0, const gtsam::Key key_x1, const NoiseModel& cost_model,
                        const Eigen::Index x0_dim, const Eigen::Index x1_dim, const double h)
    : Base(cost_model, key_x0, key_x1)
    , _x0_ptr()
    , _x1_ptr()
    , _partial_x0([&](const X0& x0) { return compute_error(x0, *_x1_ptr); })
    , _partial_x1([&](const X1& x1) { return compute_error(*_x0_ptr, x1); })
    , _derivative_x0(_partial_x0, h, x0_dim, x0_dim)
    , _derivative_x1(_partial_x1, h, x1_dim, x0_dim)
  {
  }

  template <Eigen::Index DimX0 = Dim0, Eigen::Index DimX1 = Dim1,
            std::enable_if_t<(DimX0 != Eigen::Dynamic) && (DimX1 != Eigen::Dynamic), bool> = true>
  noise_model_2factor_t(const gtsam::Key key_x0, const gtsam::Key key_x1, const NoiseModel& cost_model, const double h)
    : noise_model_2factor_t(key_x0, key_x1, cost_model, Dim0, Dim1, h)
  {
  }

  virtual ~noise_model_2factor_t()
  {
  }

  virtual Eigen::VectorXd evaluateError(const X0& x0, const X1& x1, boost::optional<Eigen::MatrixXd&> H0 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H1 = boost::none) const override
  {
    const Error error{ compute_error(x0, x1) };
    _x0_ptr.reset(&x0);
    _x1_ptr.reset(&x1);
    if (H0)
    {
      *H0 = _derivative_x0(x0);
    }

    if (H1)
    {
      *H1 = _derivative_x1(x1);
    }

    _x0_ptr.release();
    _x1_ptr.release();
    return error;
  }

  virtual X0 predict(const X1& x1) const = 0;

  virtual Error compute_error(const X0& x0, const X1& x1) const
  {
    return predict(x1) - x0;
  }

private:
  mutable std::unique_ptr<const X0> _x0_ptr;
  mutable std::unique_ptr<const X1> _x1_ptr;

  const PartialX0 _partial_x0;
  const PartialX1 _partial_x1;

  const FirstOrderDerivativeX0 _derivative_x0;
  const FirstOrderDerivativeX1 _derivative_x1;
};
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
public:
  static constexpr Eigen::Index Dim0{ gtsam::traits<X0>::dimension };
  static constexpr Eigen::Index Dim1{ gtsam::traits<X1>::dimension };
  static constexpr Eigen::Index Dim2{ gtsam::traits<X2>::dimension };

  using Error = Eigen::Vector<double, Dim0>;
  using PartialX0 = std::function<Error(const X0&)>;
  using PartialX1 = std::function<Error(const X1&)>;
  using PartialX2 = std::function<Error(const X2&)>;
  using FirstOrderDerivativeX0 = prx::math::first_order_derivative_t<PartialX0, X0, Evaluations>;
  using FirstOrderDerivativeX1 = prx::math::first_order_derivative_t<PartialX1, X1, Evaluations>;
  using FirstOrderDerivativeX2 = prx::math::first_order_derivative_t<PartialX2, X2, Evaluations>;
  using Base = gtsam::NoiseModelFactor3<X0, X1, X2>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

public:
  noise_model_3factor_t(const gtsam::Key key_x0, const gtsam::Key key_x1, const gtsam::Key key_x2,
                        const NoiseModel& cost_model, const Eigen::Index x0_dim, const Eigen::Index x1_dim,
                        const Eigen::Index x2_dim, const double h)
    : Base(cost_model, key_x0, key_x1, key_x2)
    , _x0_ptr()
    , _x1_ptr()
    , _x2_ptr()
    , _partial_x0([&](const X0& x0) { return compute_error(x0, *_x1_ptr, *_x2_ptr); })
    , _partial_x1([&](const X1& x1) { return compute_error(*_x0_ptr, x1, *_x2_ptr); })
    , _partial_x2([&](const X2& x2) { return compute_error(*_x0_ptr, *_x1_ptr, x2); })
    , _derivative_x0(_partial_x0, h, x0_dim, x0_dim)
    , _derivative_x1(_partial_x1, h, x1_dim, x0_dim)
    , _derivative_x2(_partial_x2, h, x2_dim, x0_dim)
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

  virtual Eigen::VectorXd evaluateError(const X0& x0, const X1& x1, const X2& x2,
                                        boost::optional<Eigen::MatrixXd&> H0 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H1 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H2 = boost::none) const override
  {
    const Error error{ compute_error(x0, x1, x2) };
    _x0_ptr.reset(&x0);
    _x1_ptr.reset(&x1);
    _x2_ptr.reset(&x2);
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
    _x0_ptr.release();
    _x1_ptr.release();
    _x2_ptr.release();
    return error;
  }

  virtual X0 predict(const X1& x1, const X2& x2) const = 0;

  virtual Error compute_error(const X0& x0, const X1& x1, const X2& x2) const
  {
    return predict(x1, x2) - x0;
  }

private:
  mutable std::unique_ptr<const X0> _x0_ptr;
  mutable std::unique_ptr<const X1> _x1_ptr;
  mutable std::unique_ptr<const X2> _x2_ptr;

  const PartialX0 _partial_x0;
  const PartialX1 _partial_x1;
  const PartialX2 _partial_x2;

  const FirstOrderDerivativeX0 _derivative_x0;
  const FirstOrderDerivativeX1 _derivative_x1;
  const FirstOrderDerivativeX2 _derivative_x2;
};

template <typename X0, typename X1, typename X2, typename X3, Evals Evaluations = 4>
class noise_model_4factor_t : public gtsam::NoiseModelFactor4<X0, X1, X2, X3>
{
public:
  static constexpr Eigen::Index Dim0{ gtsam::traits<X0>::dimension };
  static constexpr Eigen::Index Dim1{ gtsam::traits<X1>::dimension };
  static constexpr Eigen::Index Dim2{ gtsam::traits<X2>::dimension };
  static constexpr Eigen::Index Dim3{ gtsam::traits<X3>::dimension };

  using Error = Eigen::Vector<double, Dim0>;
  using PartialX0 = std::function<Error(const X0&)>;
  using PartialX1 = std::function<Error(const X1&)>;
  using PartialX2 = std::function<Error(const X2&)>;
  using PartialX3 = std::function<Error(const X3&)>;
  using FirstOrderDerivativeX0 = prx::math::first_order_derivative_t<PartialX0, X0, Evaluations>;
  using FirstOrderDerivativeX1 = prx::math::first_order_derivative_t<PartialX1, X1, Evaluations>;
  using FirstOrderDerivativeX2 = prx::math::first_order_derivative_t<PartialX2, X2, Evaluations>;
  using FirstOrderDerivativeX3 = prx::math::first_order_derivative_t<PartialX3, X3, Evaluations>;
  using Base = gtsam::NoiseModelFactor4<X0, X1, X2, X3>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

  using Key = gtsam::Key;

public:
  noise_model_4factor_t(const Key key_x0, const Key key_x1, const Key key_x2, const Key key_x3,
                        const NoiseModel& cost_model, const Eigen::Index x0_dim, const Eigen::Index x1_dim,
                        const Eigen::Index x2_dim, const Eigen::Index x3_dim, const double h)
    : Base(cost_model, key_x0, key_x1, key_x2, key_x3)
    , _x0_ptr()
    , _x1_ptr()
    , _x2_ptr()
    , _x3_ptr()
    , _partial_x0([&](const X0& x0) { return compute_error(x0, *_x1_ptr, *_x2_ptr, *_x3_ptr); })
    , _partial_x1([&](const X1& x1) { return compute_error(*_x0_ptr, x1, *_x2_ptr, *_x3_ptr); })
    , _partial_x2([&](const X2& x2) { return compute_error(*_x0_ptr, *_x1_ptr, x2, *_x3_ptr); })
    , _partial_x3([&](const X3& x3) { return compute_error(*_x0_ptr, *_x1_ptr, *_x2_ptr, x3); })
    , _derivative_x0(_partial_x0, h, x0_dim, x0_dim)
    , _derivative_x1(_partial_x1, h, x1_dim, x0_dim)
    , _derivative_x2(_partial_x2, h, x2_dim, x0_dim)
    , _derivative_x3(_partial_x3, h, x3_dim, x0_dim)
  {
  }

  template <Eigen::Index DimX0 = Dim0, Eigen::Index DimX1 = Dim1, Eigen::Index DimX2 = Dim2, Eigen::Index DimX3 = Dim3,
            std::enable_if_t<(DimX0 != Eigen::Dynamic) && (DimX1 != Eigen::Dynamic) && (DimX2 != Eigen::Dynamic) &&
                                 (DimX3 != Eigen::Dynamic),
                             bool> = true>
  noise_model_4factor_t(const Key key_x0, const Key key_x1, const Key key_x2, const Key key_x3,
                        const NoiseModel& cost_model, const double h)
    : noise_model_4factor_t(key_x0, key_x1, key_x2, key_x3, cost_model, Dim0, Dim1, Dim2, Dim3, h)
  {
  }

  virtual ~noise_model_4factor_t()
  {
  }

  virtual Eigen::VectorXd evaluateError(const X0& x0, const X1& x1, const X2& x2, const X3& x3,
                                        boost::optional<Eigen::MatrixXd&> H0 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H1 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H2 = boost::none,
                                        boost::optional<Eigen::MatrixXd&> H3 = boost::none) const override
  {
    const Error error{ compute_error(x0, x1, x2, x3) };
    _x0_ptr.reset(&x0);
    _x1_ptr.reset(&x1);
    _x2_ptr.reset(&x2);
    _x3_ptr.reset(&x3);
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

    if (H3)
    {
      *H3 = _derivative_x3(x3);
    }

    _x0_ptr.release();
    _x1_ptr.release();
    _x2_ptr.release();
    _x3_ptr.release();

    return error;
  }

  virtual X0 predict(const X1& x1, const X2& x2, const X3& x3) const = 0;

  virtual Error compute_error(const X0& x0, const X1& x1, const X2& x2, const X3& x3) const
  {
    return predict(x1, x2, x3) - x0;
  }

private:
  mutable std::unique_ptr<const X0> _x0_ptr;
  mutable std::unique_ptr<const X1> _x1_ptr;
  mutable std::unique_ptr<const X2> _x2_ptr;
  mutable std::unique_ptr<const X3> _x3_ptr;

  const PartialX0 _partial_x0;
  const PartialX1 _partial_x1;
  const PartialX2 _partial_x2;
  const PartialX3 _partial_x3;

  const FirstOrderDerivativeX0 _derivative_x0;
  const FirstOrderDerivativeX1 _derivative_x1;
  const FirstOrderDerivativeX2 _derivative_x2;
  const FirstOrderDerivativeX3 _derivative_x3;
};

// namespace gtsam
// {
// template <Eigen::Index Dim>
// struct traits<Eigen::Vector<double, Dim>>
// {
//   using dimension = Dim;
//   static constexpr int GetDimension(const Eigen::Vector<double, Dim>&)
//   {
//     return Dim;
//   }
// };
// }  // namespace gtsam
}  // namespace fg
}  // namespace prx