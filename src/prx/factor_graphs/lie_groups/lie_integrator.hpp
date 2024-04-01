#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include <gtsam/base/OptionalJacobian.h>
#include "prx/factor_graphs/lie_groups/lie_operators.hpp"
#include "prx/utilities/math/first_order_derivative.hpp"

namespace prx
{
namespace fg
{

/**
 * Integration class, the GTSAM class is intedead for IMU
 * this one is for forward propagation / planning /control
 * (No imu bias, different params)
 */
template <typename X, typename Xdot, Eigen::Index RowsCols = X::RowsAtCompileTime>
class lie_integrator_t
{
  using Model = lie_integrator_t<X, Xdot, RowsCols>;
  using Derivative = prx::math::first_order_derivative_t<Model, X, 2, 0>;

public:
  lie_integrator_t() = default;

  X operator()(const X& xi, const Xdot& xdot_i, double dt) const
  {
    const Xdot xdot_dt{ xdot_i * dt };
    const X exmap_xdot_dt{ X::template expmap<Xdot>(xdot_dt) };
    const X xj{ xi * exmap_xdot_dt };
    return xj;
  }
  // f(x_i, \dot{x}_i) x_{i+1} = x_i * expmap(\dot{x}_i * dt)
  static X propagate(const X& xi, const Xdot& xdot_i, double dt,  // no-lint
                     gtsam::OptionalJacobian<RowsCols, RowsCols> Hxi = boost::none,
                     gtsam::OptionalJacobian<RowsCols, RowsCols> Hxdot = boost::none)
  {
    static const Derivative derivative(dt);
    static const Model model{};
    if (Hxi)
    {
      *Hxi = derivative(xi);
    }
    if (Hxdot)
    {
      // lie_operators::leibniz_rule(*Hxi, xi, exmap_xdot_dt, identity, Hexpmap);
    }
    return model(xi, xdot_i, dt);
  }

  // syntatic sugar for FG-terms
  // template <typename X, typename Xdot>
  static inline const X predict(const X& x_i, const Xdot& xdot_i, double dt)
  {
    return propagate(x_i, xdot_i, dt);
  }
};

}  // namespace fg
}  // namespace prx
