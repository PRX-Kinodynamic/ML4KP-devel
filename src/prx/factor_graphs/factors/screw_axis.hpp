#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include <gtsam/geometry/SO3.h>
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
  using Rotation = Eigen::Matrix3d;
  screw_axis_t(Base vals) : Eigen::Vector<double, 6>(vals), _omega((*this).data(), 3, 1), _v((*this).data() + 3, 3, 1)
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

  static SE3_t exp(const screw_axis_t& s)
  {
    SE3_t tr(SE3_t::Base::Identity());
    tr.linear() = gtsam::SO3::Expmap(s.omega()).matrix();
    tr.translation() = gtsam::SO3::LogmapDerivative(s.omega()) * s.v();

    return tr;
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
  // Convinient maps to the two components of the screw axis
  Eigen::Map<Eigen::Vector3d> _omega;
  Eigen::Map<Eigen::Vector3d> _v;
};
}  // namespace fg
}  // namespace prx