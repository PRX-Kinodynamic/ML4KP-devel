#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include "prx/factor_graphs/factors/lie_operators.hpp"

namespace prx
{
namespace fg
{

// A Screw axis S = [\omega, v], where a Twist V = S \dot{\Theta}
class screw_axis_t : public Eigen::Vector<double, 6>
{
public:
  static constexpr Eigen::Index Dim = 6;

  using Base = Eigen::Vector<double, 6>;
  using Twist = Eigen::Vector<double, 6>;
  screw_axis_t(Eigen::Vector<double, 6> vals)
    : Eigen::Vector<double, 6>(vals), _omega((*this).data(), 3, 1), _v((*this).data() + 3, 3, 1)
  {
  }
  screw_axis_t() : screw_axis_t(Eigen::Vector<double, 6>::Zero())
  {
  }

  virtual ~screw_axis_t()
  {
  }

  using Base::operator=;

  template <typename Omega, typename Velocity>
  static screw_axis_t from_twist(const Omega omega, const Velocity velocity)
  {
    return from_twist((Twist() << omega, velocity).finished());
  }

  static screw_axis_t from_twist(const Twist twist)
  {
    screw_axis_t screw(twist);
    screw = screw / screw.theta_dot();
    // screw = screw * 5;
    return screw;
  }

  // Get the twist representation of this screw_axis
  Twist twist() const
  {
    return Twist((*this) * theta_dot());
  }

  inline bool is_omega_zero() const
  {
    return _omega.isApproxToConstant(0.0);
  }
  // \dot{\Theta} such that S \cdot \dot{\Theta} = V
  double theta_dot() const
  {
    return is_omega_zero() ? _v.norm() : _omega.norm();
  }

  static SE3_t exp(const screw_axis_t& s, const double dt)
  {
    SE3_t transform{ SE3_t::Base::Identity() };
    const double theta{ s.theta_dot() * dt };
    const Eigen::Matrix3d w_hat{ lie_operators::hat(s.omega()) };
    transform.linear() = w_hat * theta;
    transform.translation() = G(w_hat, theta) * s.v();
    return transform;
  }

  Eigen::Map<Eigen::Vector3d> omega() const
  {
    return _omega;
  }

  Eigen::Map<Eigen::Vector3d> v() const
  {
    return _v;
  }

  Eigen::Map<Eigen::Vector3d>& omega()
  {
    return _omega;
  }

  Eigen::Map<Eigen::Vector3d>& v()
  {
    return _v;
  }

  friend std::ostream& operator<<(std::ostream& os, const screw_axis_t& screw)
  {
    os << "( " << screw.omega().transpose() << " " << screw.v().transpose() << " )";
    return os;
  }

private:
  static Eigen::Matrix3d G(const Eigen::Matrix3d w_hat, const double theta)
  {
    const Eigen::Matrix3d g0{ Eigen::Matrix3d::Identity() * theta };
    const Eigen::Matrix3d g1{ (1 - std::cos(theta)) * w_hat };
    const Eigen::Matrix3d g2{ (theta - std::sin(theta)) * w_hat * w_hat };
    return g0 + g1 + g2;
  }
  // Convinient maps to the two components of the screw axis
  Eigen::Map<Eigen::Vector3d> _omega;
  Eigen::Map<Eigen::Vector3d> _v;
};
}  // namespace fg
}  // namespace prx