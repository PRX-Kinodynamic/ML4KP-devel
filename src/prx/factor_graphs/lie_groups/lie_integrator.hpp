#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include <gtsam/base/OptionalJacobian.h>
#include "prx/factor_graphs/lie_groups/lie_operators.hpp"
#include "prx/factor_graphs/factors/noise_model_factors.hpp"

namespace prx
{
namespace fg
{
//
// Numerical integrator that does the equivalent of:
// x_{t+1} <- x_t + \dot{x} dt
// But using lie-groups operators, this is
// x_{t+1} = f(x_t, \dot{x}_t)  = x_t * expmap(\dot{x}_t * dt)
// For a given prediction \hat{x}_{t+1}, the error is then:
// f(x_t, \dot{x}_t) - \hat{x}_{t+1}
template <typename X, typename Xdot>
class lie_integrator_t  //: public gtsam::NoiseModelFactor3<X, Xdot, X>
{
  using Model = lie_integrator_t<X, Xdot>;
  using Derivative = prx::math::first_order_derivative_t<Model, X, 2, 0>;
  static constexpr Eigen::Index DimX{ gtsam::traits<X>::dimension };
  static constexpr Eigen::Index DimXdot{ gtsam::traits<Xdot>::dimension };

public:
  lie_integrator_t() = default;

  static X integrate(const X& xi, const Xdot& xdot_i, const double dt,
                     gtsam::OptionalJacobian<DimX, DimX> Hx = boost::none,
                     gtsam::OptionalJacobian<DimX, DimXdot> Hxdot = boost::none)
  {
    gtsam::OptionalJacobian<DimX, DimXdot> Hexmap{ Hxdot };
    const Xdot xdot_dt{ xdot_i * dt };
    const X exmap_xdot_dt{ X::Expmap(xdot_dt, Hexmap) };
    // const X xj{ xi * exmap_xdot_dt };
    const X xj{ gtsam::traits<X>::Compose(xi, exmap_xdot_dt, Hx, Hxdot) };
    // static Class Compose(const Class& m1, const Class& m2, //
    //       ChartJacobian H1 = boost::none, ChartJacobian H2 = boost::none) {
    //     return m1.compose(m2, H1, H2);
    //   }

    return xj;
  }
};

//: public noise_model_3factor_t<X, X, Xdot>
template <typename X, typename Xdot>
class lie_integration_factor_t : public gtsam::NoiseModelFactor3<X, X, Xdot>
{
  using Base = gtsam::NoiseModelFactor3<X, X, Xdot>;
  using LieIntegrator = lie_integrator_t<X, Xdot>;
  using NoiseModel = gtsam::noiseModel::Base::shared_ptr;

public:
  lie_integration_factor_t(const gtsam::Key key_xt1, const gtsam::Key key_xt0, const gtsam::Key key_xdot,
                           const NoiseModel& cost_model, const double h)
    : Base(cost_model, key_xt1, key_xt0, key_xdot), _h(h)
  {
  }

  // virtual X0 predict(const X1& x1, const X2& x2) const = 0;
  virtual X predict(const X& xi, const Xdot& xdot_i) const
  {
    return LieIntegrator::integrate(xi, xdot_i, _h);
  }

private:
  const double _h;
};

}  // namespace fg
}  // namespace prx
