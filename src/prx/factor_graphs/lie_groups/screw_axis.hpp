#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/geometry/SO3.h>
#include <gtsam/geometry/Pose3.h>
#include "prx/factor_graphs/lie_groups/lie_operators.hpp"
#include "prx/factor_graphs/lie_groups/se3.hpp"
namespace prx
{
namespace fg
{
// A Screw axis S = [\omega, v], where a Twist V = S \dot{\Theta}
class screw_axis_t
{
public:
  static constexpr Eigen::Index Dim = 6;
  static constexpr Eigen::Index dimension = 6;

  using OmegaV = Eigen::Vector<double, 6>;
  using Twist = Eigen::Vector<double, 6>;
  using Rotation = Eigen::Matrix3d;

  screw_axis_t() : screw_axis_t(Eigen::Vector<double, 6>::Zero())
  {
  }

  screw_axis_t(const screw_axis_t&) = default;

  // This constructor allows you to construct screw_axis_t from Eigen expressions
  template <typename OtherDerived>
  screw_axis_t(const Eigen::MatrixBase<OtherDerived>& other)
    : _omega_v(other), _omega(_omega_v.data(), 3, 1), _v(_omega_v.data() + 3, 3, 1)
  {
  }

  virtual ~screw_axis_t()
  {
  }

  double& operator[](const std::size_t& idx)
  {
    return _omega_v[idx];
  }

  double operator[](const std::size_t& idx) const
  {
    return _omega_v[idx];
  }

  OmegaV vector() const
  {
    return _omega_v;
  }

  OmegaV& vector()
  {
    return _omega_v;
  }

  operator Eigen::Vector<double, 6>() const
  {
    return _omega_v;
  }

  // This method allows you to assign Eigen expressions to screw_axis_t
  template <typename OtherDerived>
  screw_axis_t& operator=(const Eigen::MatrixBase<OtherDerived>& other)
  {
    _omega_v = other;
    return *this;
  }

  template <typename Arithmetic, std::enable_if_t<std::is_arithmetic_v<Arithmetic>, bool> = true>
  screw_axis_t operator/(const Arithmetic& arithmetic) const
  {
    return screw_axis_t{ _omega_v / arithmetic };
  }

  template <typename OtherDerived>
  screw_axis_t operator/(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    return screw_axis_t{ _omega_v / other };
  }

  template <typename Arithmetic, std::enable_if_t<std::is_arithmetic_v<Arithmetic>, bool> = true>
  screw_axis_t operator*(const Arithmetic& other) const
  {
    return screw_axis_t{ _omega_v * other };
  }

  template <typename OtherDerived>
  screw_axis_t operator*(const Eigen::MatrixBase<OtherDerived>& other) const
  {
    return screw_axis_t{ _omega_v * other };
  }

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
    return Twist(_omega_v * theta_dot());
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

  template <typename SE3>
  static SE3 expmap(const screw_axis_t& s, gtsam::OptionalJacobian<6, 6> H = boost::none)
  {
    if (H)
    {
      (*H).block<3, 3>(0, 0) = gtsam::SO3::ExpmapDerivative(s.omega());
    }
    const gtsam::Rot3 rotation{ gtsam::SO3::Expmap(s.omega()) };
    const Eigen::Vector3d translation{ gtsam::SO3::LogmapDerivative(s.omega()) * s.v() };
    // PRX_DEBUG_VAR_1(s.omega().dot(s.omega()));
    // PRX_DEBUG_VAR_1(s.omega());
    // PRX_DEBUG_VAR_1(rotation);
    // PRX_DEBUG_VAR_1(rotation.toQuaternion());
    const SE3 se3{ rotation.toQuaternion(), translation };
    return se3;
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
    os << screw.omega().transpose() << " " << screw.v().transpose();
    return os;
  }

  void print(const std::string& str = "") const
  {
    std::cout << str << " " << _omega_v.transpose();
  }

  bool equals(const screw_axis_t& other, double tol = 1e-8) const
  {
    return omega().isApprox(other.omega(), tol) && v().isApprox(other.v(), tol);
  }

private:
  // Convinient maps to the two components of the screw axis
  OmegaV _omega_v;
  Eigen::Map<Eigen::Vector3d> _omega;
  Eigen::Map<Eigen::Vector3d> _v;
};

}  // namespace fg
}  // namespace prx
namespace gtsam
{

template <>
struct traits<prx::fg::screw_axis_t> : public gtsam::Testable<prx::fg::screw_axis_t>,
                                       public gtsam::internal::VectorSpaceImpl<prx::fg::screw_axis_t, 6>
{
  static int GetDimension(const prx::fg::screw_axis_t&)
  {
    return 6;
  }
};
}  // namespace gtsam