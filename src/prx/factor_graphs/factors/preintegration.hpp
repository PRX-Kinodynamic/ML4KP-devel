#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include <gtsam/base/OptionalJacobian.h>
namespace prx
{
namespace fg
{

/**
 * Preintegration class, the GTSAM class is intedead for IMU
 * this one is for forward propagation / planning /control
 * (No imu bias, different params)
 */
template <typename X, typename Xdot>
class preintegration_t
{
  static constexpr Eigen::Index Xdim{ X::Dim };
  static constexpr Eigen::Index XdotDim{ Xdot::Dim };

public:
  template <Eigen::Index InputDim = Xdim, std::enable_if_t<(Xdim == XdotDim), bool> = true>
  preintegration_t(const double dt) : _dTij(dt)
  {
  }

  virtual ~preintegration_t()
  {
  }

  // virtual void resetIntegration() = 0;

  // delta t between steps i and j
  inline double deltaT() const
  {
    return _dTij;
  }

  friend std::ostream& operator<<(std::ostream& os, const preintegration_t& pim)
  {
    os << pim._dTij << "\n";
    return os;
  }
  virtual void print(const std::string s = "") const
  {
    if (s.size() > 0)
    {
      std::cout << s << " ";
    }
    std::cout << *this;
  }

  // x_j <- x_0 exp()
  const X propagate(const X& x_i, const Xdot& xdot_i) const
  {
    const X x_j{ x_i * Xdot::exp(xdot_i, _dTij) };
    PRX_DEBUG_VAR_1(x_i);
    PRX_DEBUG_VAR_1(xdot_i);
    PRX_DEBUG_VAR_1(Xdot::exp(xdot_i, _dTij));
    PRX_DEBUG_VAR_1(x_j);
    return x_j;
  }

  /// Calculate error given a state xp_j
  X error(const X& xp_j, const X& x_i, const Xdot& xdot_i, gtsam::OptionalJacobian<Xdim, Xdim> H0,
          gtsam::OptionalJacobian<Xdim, Xdim> H1, gtsam::OptionalJacobian<Xdim, XdotDim> H2) const
  {
    const X x_j{ propagate(x_i, xdot_i, H1, H2) };
    const X error{ x_j - xp_j };

    // H0: Derivative of error with respect to xp_j
    if (H0)
    {
      *H0 = X::Jacobian(error, xp_j);
    }
    // Derivative of error with respect to x_i
    if (H1)
    {
      *H1 = X::Jacobian(error, x_i);
    }
    // Derivative of error with respect to xdot_i
    if (H2)
    {
      *H2 = X::Jacobian(error, xdot_i);
    }
    return error;
  }

protected:
  // Time interval from i to j
  double _dTij;
};

}  // namespace fg
}  // namespace prx
