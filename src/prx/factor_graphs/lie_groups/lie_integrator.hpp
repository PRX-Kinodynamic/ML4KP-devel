#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include <gtsam/base/OptionalJacobian.h>
namespace prx
{
namespace fg
{

/**
 * Integration class, the GTSAM class is intedead for IMU
 * this one is for forward propagation / planning /control
 * (No imu bias, different params)
 */
class lie_integrator_t
{
public:
  // x_j <- x_0 exp()
  template <typename X, typename Xdot>
  static const X propagate(const X& xi, const Xdot& xdot_i, double dt,  // no-lint
                           gtsam::OptionalJacobian<6, 6> Hxi = boost::none,
                           gtsam::OptionalJacobian<6, 6> Hxdot = boost::none)
  {
    const Xdot xdot_dt{ xdot_i * dt };
    const X exmap_xdot_dt{ Xdot::template expmap<X>(xdot_dt, Hxdot) };
    const X xj{ xi * exmap_xdot_dt };
    // PRX_DEBUG_VAR_2(xdot_i, dt);
    // PRX_DEBUG_VAR_1(xdot_dt);
    // PRX_DEBUG_VAR_1(xi);
    // PRX_DEBUG_VAR_1(exmap_xdot_dt);
    // PRX_DEBUG_VAR_1(xj);
    if (Hxi)
    {
      *Hxi = Eigen::Matrix<double, 6, 6>::Identity();
    }
    return xj;
  }

  // syntatic sugar for FG-terms
  template <typename X, typename Xdot>
  static inline const X predict(const X& x_i, const Xdot& xdot_i, double dt)
  {
    return propagate(x_i, xdot_i, dt);
  }
};

}  // namespace fg
}  // namespace prx
