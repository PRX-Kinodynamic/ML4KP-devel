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
template <typename X, typename Xdot>
class euler_integration_factor_t : public gtsam::NoiseModelFactor3<X, X, Xdot>
{
  using Base = gtsam::NoiseModelFactor3<X, X, Xdot>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;
  static constexpr Eigen::Index DimX{ gtsam::traits<X>::dimension };
  static constexpr Eigen::Index DimXdot{ gtsam::traits<Xdot>::dimension };

  using DerivativeX = Eigen::Matrix<double, DimX, DimX>;
  using DerivativeXdot = Eigen::Matrix<double, DimXdot, DimXdot>;
  using OptDeriv = boost::optional<Eigen::MatrixXd&>;

public:
  euler_integration_factor_t(const gtsam::Key key_xt1, const gtsam::Key key_xt0, const gtsam::Key key_xdot,
                             const NoiseModel& cost_model, const double h)
    : Base(cost_model, key_xt1, key_xt0, key_xdot), _h(h), _negative_identity(-1 * DerivativeX::Identity())
  {
  }

  static X integrate(const X& xi, const Xdot& xdot_i, const double dt, OptDeriv Hx = boost::none,
                     OptDeriv Hxdot = boost::none)
  {
    return predict(xi, xdot_i, dt, Hx, Hxdot);
  }

  static X predict(const X& x, const Xdot& xdot, const double dt, OptDeriv Hx = boost::none,
                   OptDeriv Hxdot = boost::none)
  {
    // clang-format off
    if (Hx){ *Hx = DerivativeX::Identity(); }
    if (Hxdot){ *Hxdot = dt * DerivativeXdot::Identity(); }
    // clang-format on
    return x + xdot * dt;
  }

  // x1_predicted <- x0 + xdot dt
  // Error is: x1_predicted - x1_observed
  virtual Eigen::VectorXd evaluateError(const X& x1, const X& x0, const Xdot& xdot, OptDeriv H1 = boost::none,
                                        OptDeriv H0 = boost::none, OptDeriv Hdot = boost::none) const override
  {
    const X prediction{ predict(x0, xdot, _h, H0, Hdot) };
    // X1_p (-) x1 => Eq. 26 from "A micro Lie theory [...]" https://arxiv.org/pdf/1812.01537.pdf
    const Eigen::VectorXd error{ prediction - x1 };

    // clang-format off
    if (H1) { *H1 = _negative_identity; }
    // clang-format on

    return error;
  }

private:
  const double _h;
  const DerivativeX _negative_identity;
};
}  // namespace fg
}  // namespace prx